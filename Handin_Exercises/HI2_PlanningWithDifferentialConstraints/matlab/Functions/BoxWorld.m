classdef BoxWorld < handle
    % Class for describing a world with given boundaries and box-shaped
    % obstacles.
    properties
        xmin = 0;
        xmax = 0;
        ymin = 0;
        ymax = 0;
        fig;
        boxes = [];
        x_obst = [];
        y_obst = [];
        st_sp;
    end
    
    methods
        function obj = BoxWorld(lattice)
            % Create a BoxWorld object with the given lattice
            obj.st_sp = state_space_from_lattice(lattice);
            obj.xmin = min(lattice{1});
            obj.xmax = max(lattice{1});
            obj.ymin = min(lattice{2});
            obj.ymax = max(lattice{2});
            obj.fig = NaN;
            obj.boxes = [];
            obj.x_obst = [];
            obj.y_obst = [];
        end

        function n = num_nodes(obj)
            % Get the total number of nodes in the state space
            n = size(obj.st_sp, 2);
        end
        
        function add_box(obj, x, y, W1, W2, fill_box)
            % Add a box-shaped constraint in the world
            if nargin < 6
                fill_box = true;
            end
            obj.boxes(end+1, :) = [x, y, W1, W2, fill_box];            
            obj.x_obst(end+1, :) = [x, x + W1];
            obj.y_obst(end+1, :) = [y, y + W2];
        end

        function register_figure(obj, fig)
            obj.fig = fig;
        end

        function draw(obj, varargin)
            % Draw the obstacles in the world in the figure
            if nargin < 2
                args = 'r';
            else
                args = varargin;
            end
            obj.redraw_boxes(args);
        end
        
        function redraw_boxes(obj, varargin)
            hold_state = ishold;
            hold on
            for ii=1:size(obj.boxes, 1)
                draw_box(obj.boxes(ii, :), varargin{:});
            end
            if ~hold_state
                hold off
            end
        end
        
        function c = in_bound(obj, point)
            % Check if a given point is within the world-model boundaries
            c = false;
            if point(1) >= obj.xmin && point(1) <= obj.xmax && ...
               point(2) >= obj.ymin && point(2) <= obj.ymax
                c = true;
            end
        end
        
        function of = obstacle_free(obj, p)
            % Check if any of the given points are in collision with 
            % obstacles in the world, return 0 if so or 1 otherwise
            coll = 0;

            for ii = 1:size(p, 2)
                if obstacle_check(p(1, ii),p(2, ii), obj.x_obst, obj.y_obst) == 1
                    coll = 1;
                    break;
                end
            end
            of = 1-coll;
        end   
        
    end    
end

function collision = obstacle_check(x, y, x_obst, y_obst)
    % Help function to function obstacle_free, to check collision for a 
    % single point x,y
    collision = 0;
    for i = 1:size(x_obst,1)
        if (x > x_obst(i,1) && x < x_obst(i,2)) && (y > y_obst(i,1) && y < y_obst(i,2))
            collision = 1;
            return;
        end
    end
end

function draw_box(b, varargin)   
    % Help function to function draw, for drawing a box in the figure
    x0 = b(1);
    y0 = b(2);
    W1 = b(3);
    W2 = b(4);
    fill_box = b(5);
    if fill_box
        fill([x0, x0 + W1, x0 + W1, x0, x0], ...
             [y0, y0, y0 + W2, y0 + W2, y0], varargin{:});
    else
        plot([x0, x0 + W1, x0 + W1, x0, x0], ...
             [y0, y0, y0 + W2, y0 + W2, y0], varargin{:});
    end
end

function st_sp = state_space_from_lattice(lattice)
    % Create a matrix st_sp with all states in the world, given the 
    % specified lattice parameters. In the lattice planning, this 3 x N
    % matrix is used as a mapping between node number and actual
    % coordinates, where the column number is the node number.
    if length(lattice) == 1
        st_sp = lattice{1};
    else
        st_sp_1 = state_space_from_lattice(lattice(2:end));
        N = size(st_sp_1, 2);
        st_sp = [];
        
        for ii=1:length(lattice{1})
            st_sp = [st_sp, [lattice{1}(ii)*ones(1, N); st_sp_1]];
        end
    end
end

