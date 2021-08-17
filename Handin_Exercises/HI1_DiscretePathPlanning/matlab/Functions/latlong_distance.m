function d = LatLongDistance(p1, p2)
% LATLONGDISTANCE  Compute distance between two points defined in lat/long.
%
%  Example:
%    d = LatLongDistance(p1, p2)
%  where a point p is described as p=(lat, long). Returns distance in
%  meters.


  % Haversine distance
  radius = 6371; % km
  lat1 = p1(2)*pi/180;
  lat2 = p2(2)*pi/180;
  lon1 = p1(1)*pi/180;
  lon2 = p2(1)*pi/180;
  deltaLat  =lat2-lat1;
  deltaLon = lon2-lon1;
  a = sin((deltaLat)/2)^2 + cos(lat1)*cos(lat2) * sin(deltaLon/2)^2;
  c = 2*atan2(sqrt(a),sqrt(1-a));
  d = radius*c;    
  d = d*1e3; % Return in m
end
