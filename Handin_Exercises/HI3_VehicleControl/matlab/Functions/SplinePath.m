classdef SplinePath < handle
    properties
        path = [];
        length;
    end
  
    properties (Access=private)
        pp_x;
        pp_y;
        pp_dx;
        pp_dy;
        pp_ddx;
        pp_ddy;
        pp_c;
        pp_c_lim;
        pp_dc;    
    end
    methods
        function obj = SplinePath(p)
            % Create a path object with spline interpolation.
            %
            %  obj = SplinePath(points)
            %
            %  Input:
            %    points - A Nx2 matrix with (x,y) coordinates on the path.
            %
            %  Output:
            %    obj - SplinePath object
            obj.path = p;

            si = [0;cumsum(sqrt(sum(diff(p, 1).^2, 2)))];
            obj.length = max(si);
            
            obj.pp_x = spline(si, p(:, 1));
            obj.pp_y = spline(si, p(:, 2));
            obj.pp_dx = der_spline(obj.pp_x);
            obj.pp_dy = der_spline(obj.pp_y);
            obj.pp_ddx = der_spline(obj.pp_dx);
            obj.pp_ddy = der_spline(obj.pp_dy);

            dfx = ppval(obj.pp_dx, si);
            dfy = ppval(obj.pp_dy, si);
            ddfx = ppval(obj.pp_ddx, si);
            ddfy = ppval(obj.pp_ddy, si);

            % ci = (dfx.*ddfy - dfy.*ddfx)./((dfx.^2 + dfy.^2).^(3/2));
            ci = (dfx.*ddfy - dfy.*ddfx);

            obj.pp_c = spline(si, ci);
            obj.pp_dc = der_spline(obj.pp_c);
            
            obj.pp_c_lim = [si(1), si(end);ci(1), ci(end)];
        end

        function r=x(obj, si)
            % Get x-coordinate for point s on path
            %
            %  x_coord = obj.x(s)
            r = ppval(obj.pp_x, si);
        end

        function r=y(obj, si)
            % Get y-coordinate for point s on path
            %
            %  y_coord = obj.y(s)
            r = ppval(obj.pp_y, si);
        end

        function r=c(obj, si)
            % Get curvature of path at point s
            %
            %  curv = obj.c(s)
            r = ppval(obj.pp_c, si);
            
            % Constant curvature extrapolation
            r(si < obj.pp_c_lim(1, 1)) = obj.pp_c_lim(2, 1);
            r(si > obj.pp_c_lim(1, 2)) = obj.pp_c_lim(2, 2);
        end
        
        function r=der_c(obj, si)
            r = ppval(obj.pp_dc, si);
        end

        function r=p(obj, si)
            % Get coordinates for point s on path
            %
            %  p_xy = obj.p(s)
            r = [ppval(obj.pp_x, si), ppval(obj.pp_y, si)];
        end

        function [h, nc] = heading(obj, si)
            % Get tangent and normal vector for path at point s
            %
            %  [tangent, normal] = obj.heading(s)
            %
            %  Input: 
            %     s - Position on the path
            % 
            %  Output: 
            %    tangent - Normalized vector tangent to the path
            %    normal - Normalized normal vector for the path
            
            h = [ppval(obj.pp_dx, si), ppval(obj.pp_dy, si)];
            h = h./sqrt(sum(h.^2,2));
            nc = [-h(:,2), h(:, 1)];      
        end

        function [si, dp] = project(obj, p, s0, ds, s_lim, verbose)
            % Project a point on the path
            %
            % This is a line-search method to find an orthogonal
            % projection of a point p on the path. This is a non-linear
            % problem with in general does not have a unique solution;
            % therefore an approximative approach is implemented.
            %
            %  [s, d] = obj.project(p, s0, ds, s_lim)
            %
            %  Input
            %    p  - The point to project
            %    s0 - Approximate position on the path (start of search)
            %    ds - Step used to expand the search space (not equal to
            %         the accuracy of the projection)
            %    s_lim - Number of expansions of the search space before
            %            admitting defeat.
            %
            %  Output
            %    s  - Position on the path of the projection
            %    d  - Distance between the point p and the projection
            
            
            function r=s_fun(si)
                [~, nc] = obj.heading(si);
                dp = p - obj.p(si);
                r = cross([dp, 0], [nc, 0]);
                r = r(end);
            end
            if nargin < 6
                verbose = false;
            end
            if nargin < 5
                s_lim = 20;
            end
            if nargin < 4
                ds = 1;
            end
            smin = s0;
            smax = s0;

            cnt_lim = s_lim/ds;
            cnt = 0;

            while sign(s_fun(smin)) == sign(s_fun(smax)) && cnt < cnt_lim
                smin = max(0, smin - ds);
                smax = min(smax + ds, obj.length);
                cnt = cnt + 1;
            end

            if cnt < cnt_lim  % Found sign change in interval, do a line-search
                options = optimset('Display','off');
                si = fsolve(@s_fun, s0, options);
            else % No sign change, evaluate boundary points and choose closest
                if verbose
                  fprintf('Warning: outside bounds');
                end
                dpmin = p - obj.p(smin);
                dpmax = p - obj.p(smax);
                if norm(dpmin) < norm(dpmax)
                    si = smin;
                else
                    si = smax;
                end
            end

            dp = p - obj.p(si);
            [hi, ~] = obj.heading(si);
            dp = cross([hi, 0], [dp, 0]);
            dp = dp(end);
        end

        function d = path_error(obj, w)
            % Compute the path error for a given path w
            %
            %  d = obj.path_error(w)
            %
            %  Input
            %    w - Trajectory where columns 1 and 2 are the x and y
            %        coordinate of the vehicle path.
            %
            %  Output
            %    d - Orthogonal distance to path at each time point.
            
            N = size(w, 1);
            d = zeros(N, 1);
            s0 = 0.0;

            for k=1:N
                p_car = w(k, :);
                [si, dk] = obj.project(p_car, s0, 1, 20);
                d(k) = dk;
                s0 = si;
            end
        end    
    end
end

function pp_der = der_spline(pp)
    pp_der = pp;
    pp_der.coefs = pp_der.coefs*diag([3, 2, 1, 0]);
    pp_der.coefs = [zeros(size(pp_der.coefs, 1), 1) pp_der.coefs(:, 1:3)];
end
