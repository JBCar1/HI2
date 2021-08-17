function [xi, u, d] = map_state_update(x, dm)
% MAP_STATE_UPDATE  Compute neighbors, and distances to a node
%
%   [xi, u, d] = map_state_update(x, dm)
%
% Input:
%     x  - Node (index)
%     dm - Distance matrix
%
%  Output:
%     xi - Array of node indices of neighbors
%     u  - Array of controls to get to neighbor; here this is not used
%          and returns NaN
%     d  - Array of distances (in m) to neighbors

  % xi = find(dm(x, :) > 0);
  % d = dm(x, xi);
  
  xi = find(dm(:, x) > 0); 
  d = dm(xi, x);  
  
  u = nan(numel(xi), 0);
end