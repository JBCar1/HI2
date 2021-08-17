classdef OpenStreetMap < handle
    properties
        distancematrix; % d = obj.distancematrix(p1_idx, p2_idx), returns empty if nodes are not neighbors
        nodeposition; % [longitude, latitude] = obj.nodeposition(idx)
    end
    properties (Hidden=true)
        figdata;
        osmfilename
        bounds;
        nodes;
        nodeindex;
        ways;
        waysXMLindex;
        timematrix;
        grid;
    end
    methods
        function obj = OpenStreetMap(filename, figname)
            if nargin >= 2
                obj.figdata.figName = figname;
            end
            try
                fprintf('Reading XML file ...');
                tree = xmlread(filename);
                fprintf('\nParsing XML file ...');
                p = parse_osm(tree);
                obj.bounds = p.bounds;
                obj.nodes = p.nodes;
                obj.ways = p.ways;
                obj.osmfilename = filename;
            catch
                error('Failed to read XML file %s.',filename);
            end      
            obj.waysXMLindex = cellfun(@(w) w.id, obj.ways);

            obj.purgenodes();
            fprintf('\nComputing distance matrix ...');

            obj.computewaylengths();
            obj.computedistancematrix();
            obj.computetimematrix();
            obj.compute_nodepositions();

            N = min(max(2,floor(2+4*(log10(double(obj.nodes.Count()))))),15);
            obj.nodepartition(N);

            if nargin >=2
                fprintf('\nReading map image file ...');
                obj.readmapimage(figname);
            end
            fprintf('\nDone!\n');
        end

    
        function info(obj)
            % INFO  Print general information about map object
            
            fprintf('OSMFile: %s\n', obj.osmfilename);
            fprintf('Number of nodes: %d\n', obj.nodes.length());
            fprintf('Number of roads: %d\n', numel(obj.ways));
            fprintf('Map bounds\n');
            fprintf('  Lon: %f - %f\n', obj.bounds.minlon, obj.bounds.maxlon);
            fprintf('  Lat: %f - %f\n', obj.bounds.minlat, obj.bounds.maxlat);

            dx = latlong_distance([obj.bounds.minlon, obj.bounds.minlat],...
                               [obj.bounds.minlon, obj.bounds.maxlat]);
            dy = latlong_distance([obj.bounds.minlon, obj.bounds.minlat],...
                               [obj.bounds.maxlon, obj.bounds.minlat]);

            fprintf('  Size: %.1f x %.1f (m)\n', dx, dy);
            if ~isempty(obj.figdata)
                fprintf('Figure file: %s\n', obj.figdata.fileName);
                fprintf('  Size: %d x %d (px)\n', size(obj.figdata.img, 2), size(obj.figdata.img, 1));
            end
        end

        function n = neighbors(obj, idx)
            % NEIGHBORS  Find all neighbors of a nod
            %  
            % Returns a list indices to nodes that are neighbor to given
            % node.
            %
            %  n = map.neighbors(node_index)
            n = find(obj.distancematrix(idx,:)>0);
        end

        function res = getmapposition(obj)
            % GETMAPPOSITION  Click in current figure for closest node in
            % map
            %
            %  Example: 
            %    map.plotmap()
            %    p = map.getmapposition()
            %    p.id
            %    p.pos
            res.id = obj.getclosestnodeindex(ginput(1));
            res.pos = obj.getnodecoordinates(res.id);
        end

    %    function waysidx = getwaysindex(obj, nodeid)
    %      waysXMLid = obj.getwaysXMLid(nodeid);
    %      [~, waysidx] = ismember(waysXMLid, obj.waysIndex);
    %   end

        function h = plotplan(obj, plan, varargin)
            % PLOTPLAN  Plots a plan in the current figure
            %
            %   map.plotplan(plan, opt_args)
            %
            % Plots a plan, a sequence of node indices, in the current
            % figure. Optional arguments, opt_args, accepts the same
            % arguments as the plot command.
            %
            % Example
            %   plan = DepthFirst(num_nodes, mission, f_next);
            %   figure(10)
            %   map.plotmap()
            %   hold on
            %   map.plotplan(df_plan.plan, 'b', 'linewidth', 2);
            %   hold off
            if nargin < 3
                plotargs = {'k', 'LineWidth', 2};
            else
                plotargs = varargin;
            end
            x = zeros(numel(plan),2);
            for k=1:numel(plan)
                x(k,:) = obj.nodes(obj.nodeindex(plan(k)));
            end
            h = plot( x(:,1), x(:,2), plotargs{:});
        end

        function plotmap(obj)
            % PLOTMAP  Plots the map in the current figure
            %
            %  Example
            %    figure(10)
            %    map.plotmap()
            if isstruct(obj.figdata)
                bb = [obj.bounds.minlon, obj.bounds.maxlon; ...
                  obj.bounds.minlat, obj.bounds.maxlat];
                image('CData', obj.figdata.img,...
                  'XData', bb(1,1:2), 'YData', bb(2,1:2))

                %Adjust the aspect ratio
                ax = axis;
                x_dist = diff(ax(1:2));
                y_dist = diff(ax(3:4));
                c_adj = cosd(mean(ax(3:4)));
                dar = [1 c_adj 1];
                pbar = [x_dist*c_adj/y_dist 1 1 ];
                set(gca, 'DataAspectRatio',dar,'PlotBoxAspectRatio',pbar);
                xlim(bb(1,:));
                ylim(bb(2,:));
            end
        end    
    end
  
    methods (Hidden=true)
        function nodeid = getclosestnodeindex(obj,p)
            xmlid = obj.getclosestnodeXMLid(p);
            [~,nodeid] = ismember(xmlid, obj.nodeindex);
        end

        function nodeid = getclosestnodeXMLid(obj,p)
            [~,lonidx] = find([1 p(1)>=obj.grid.lon],1,'last');
            [~,latidx] = find([1 p(2)>=obj.grid.lat],1,'last');
            lonidx = unique(min(max(1,lonidx-1:lonidx+1),obj.grid.N));
            latidx = unique(min(max(1,latidx-1:latidx+1),obj.grid.N));
            searchkeys = [obj.grid.part{lonidx, latidx}];
            mindist = inf;
            nodeid = 0;
            for k=1:numel(searchkeys)
                np = obj.nodes(searchkeys(k));
                if norm(p-np) < mindist
                    mindist = norm(p-np);
                    nodeid = searchkeys(k);
                end
            end
        end

        function waysid = getwaysXMLid(obj, nodeid)
            if numel(nodeid)>1
                nodeid = obj.getclosestnodeXMLid(nodeid);
            end
            waysid = [];
            for k=1:numel(obj.ways)
                if ismember(nodeid, obj.ways{k}.nodes)
                    waysid(end+1) = obj.ways{k}.id;
                end
            end      
        end
    
        function computetimematrix(obj)
            nk = obj.nodes.keys();
            nk = double([nk{:}]);      
            obj.nodeindex = nk;
            numnodes = double(obj.nodes.Count());
            obj.timematrix = zeros(numnodes, numnodes);
            for k=1:numel(obj.ways)
                w = obj.ways{k};
                [~,nIdx] = ismember(w.nodes,obj.nodeindex);
                for l=1:numel(w.nodes)-1
                    obj.timematrix(nIdx(l), nIdx(l+1)) = ...
                      (w.length(l+1)-w.length(l))/w.maxspeed/3.6;
                end
            end
            obj.timematrix = obj.timematrix + obj.timematrix';
            obj.timematrix = sparse(obj.timematrix);
        end
    
        function wayindex = getclosestway(obj, p)
            d = zeros(1,numel(obj.ways));
            for k=1:numel(obj.ways)
                w = obj.ways{k};
                dk = zeros(numel(w.nodes)-1,1);
                for l=1:numel(dk)
                    p1 = obj.nodes(w.nodes(l+1));
                    p2 = obj.nodes(w.nodes(l));
                    dk(l) = norm(p-p1-(p-p1)*(p2-p1)'/((p2-p1)*(p2-p1)')*(p2-p1));
                end
                d(k) = min(dk);
            end
            [~, wayindex] = min(d);      
        end
    
        function xy = getnodecoordinates(obj, nodeid)
            xy = obj.nodes(obj.nodeindex(nodeid));
        end
    
        function nodepartition(obj, N)
            obj.grid.lon = linspace(obj.bounds.minlon, obj.bounds.maxlon, N+1);
            obj.grid.lon = obj.grid.lon(2:end-1);
            obj.grid.lat = linspace(obj.bounds.minlat, obj.bounds.maxlat, N+1);
            obj.grid.lat = obj.grid.lat(2:end-1);    
            obj.grid.part = cell(N, N);
            obj.grid.N = N;
            for k=1:numel(obj.nodeindex)
                p = obj.nodes(obj.nodeindex(k));
                [~,lonIdx] = find([1 p(1)>=obj.grid.lon],1,'last');
                [~,latIdx] = find([1 p(2)>=obj.grid.lat],1,'last');
                obj.grid.part{lonIdx,latIdx}(end+1) = obj.nodeindex(k);
            end
        end

        function purgenodes(obj)
            % Purge nodes not on ways
            waynodes = obj.nodes.keys();
            waynodes = double([waynodes{:}]);
            for k=1:numel(obj.ways)
                waynodes = setdiff(waynodes, double(obj.ways{k}.nodes));
            end
            obj.nodes.remove(num2cell(waynodes));      
        end
    
        function compute_nodepositions(obj)
            obj.nodeposition = containers.Map('KeyType', 'int64', 'ValueType', 'any');

            n = numel(obj.nodeindex);
            for k=1:n
                obj.nodeposition(k) = obj.nodes(obj.nodeindex(k));
            end
        end
    
        function computewaylengths(obj)
            for k=1:numel(obj.ways)
                w = obj.ways{k};
                n = w.nodes;
                l = 0*n;
                for ni = 2:numel(n)
                    ll1 = obj.nodes(n(ni-1));
                    ll2 = obj.nodes(n(ni));
                    l(ni) = l(ni-1) + latlong_distance(ll1,ll2);
                end
                obj.ways{k}.length = l;
            end      
        end
    
        function readmapimage(obj, figname)
            obj.figdata.fileName = figname;
            obj.figdata.img = imread(figname);
            obj.figdata.img = flip(obj.figdata.img,1);      
        end
    
        function computedistancematrix(obj)
            nk = obj.nodes.keys();
            nk = double([nk{:}]);      
            obj.nodeindex = nk;
            numnodes = double(obj.nodes.Count());
            obj.distancematrix = zeros(numnodes, numnodes);
            for k=1:numel(obj.ways)
                w = obj.ways{k};
                [~,nIdx] = ismember(w.nodes,obj.nodeindex);
                for l=1:numel(w.nodes)-1
                    obj.distancematrix(nIdx(l), nIdx(l+1)) = w.length(l+1)-w.length(l);
                end
            end
            obj.distancematrix = (obj.distancematrix + obj.distancematrix');
            obj.distancematrix = sparse(obj.distancematrix);
        end

        function plotallways(obj, varargin)
            if nargin==1
                plotargs = {'b'};
            else 
                plotargs = varargin;
            end
            for k=1:numel(obj.ways)
                obj.PlotWay(obj.ways{k}, plotargs{:});
            end      
        end
    
        function plotway(obj, w, varargin)
            obj.plotpath(w.nodes, varargin{:});
        end
    
        function plotnode(obj, n, varargin)
            p = obj.nodes(obj.nodeindex(n));
            plot(p(1), p(2), varargin{:});
        end
    
        function plotpath(obj, nodepath, varargin)
            if nargin<3
                plotargs = {'b'};
            else 
                plotargs = varargin;
            end
            wxy = zeros(numel(nodepath), 2);
            for k=1:numel(nodepath)
                wxy(k,:) = obj.nodes(nodepath(k));
            end
            plot( wxy(:,1), wxy(:,2), plotargs{:});      
        end
    end
end

