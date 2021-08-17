function map = loadOSMmap(xmlfile, figurefile)
% LOADOSMMAP  Load OpenStreetMap object stored in xml-file
%
%   map = loadOSMmap(xmlfile, figfile)
%  
%   Load OpenStreetmap defined in xmlfile wth map bitmap in figfile. 
%   First time loading the command will parse the XML file which will take
%   some time, several minutes, and then store the parsed results in a mat-file. If a
%   mat-file exists, the parsing is skipped and the mat file is loaded
%   directly.

%  mapDir = '../Maps/';
%  mapName = strcat(mapDir, 'linkoping.osm'); 
%  mapFigure = strcat(mapDir, 'linkoping.png');

  [~, baseName, ~] = fileparts(xmlfile);
  if exist(strcat(baseName, '.mat'), 'file')
    fprintf('Pre-parsed file exist, loading ...\n');    
    map = load(strcat(baseName, '.mat'));
    map = map.map;
    map.compute_nodepositions();

  else
    fprintf('Pre-parsed file does not exist, parsing XML file\n');
    fprintf('This will take a while ...\n');
    tic;
    map = OpenStreetMap(xmlfile, figurefile);
    fprintf('Done parsing the XML file in %.2f seconds!\n', toc);
    fprintf('Saving data in file %s\n', strcat(baseName, '.mat'));  
    save(strcat(baseName, '.mat'), 'map');
  end
end