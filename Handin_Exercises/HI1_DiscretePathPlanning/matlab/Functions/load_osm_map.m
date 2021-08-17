function map = load_osm_map(xmlfile, figurefile, datadir)
% LOADOSMMAP  Load OpenStreetMap object stored in xml-file
%
%   map = load_osm_map(xmlfile, figurefile[, datadir])
%  
%   Load OpenStreetmap defined in xmlfile wth map bitmap in figfile. 
%   First time loading the command will parse the XML file which will take
%   some time, several minutes, and then store the parsed results in a mat-file. If a
%   mat-file exists, the parsing is skipped and the mat file is loaded
%   directly.
%   
%   datadir defaults to ../Maps/
%   Filenames xmlfile and figurefile without leadning path

%  mapDir = '../Maps/';
%  mapName = strcat(mapDir, 'linkoping.osm'); 
%  mapFigure = strcat(mapDir, 'linkoping.png');
    if nargin < 3
        datadir = "../Maps/";
    end
    
    [~, xml_name, ~] = fileparts(xmlfile);
    
    if ~isempty(getenv("OSM_MAP_PATH")) && ...
        exist(fullfile(getenv("OSM_MAP_PATH"), xml_name, '.mat'), 'file')
        fprintf('Pre-parsed file exist, loading ...\n');    
        map = load(fullfile(getenv("OSM_MAP_PATH"), '.mat'));
        map = map.map;
        map.compute_nodepositions();
    elseif exist(strcat(xml_name, '.mat'), 'file')
        fprintf('Pre-parsed file exist, loading ...\n');    
        map = load(strcat(xml_name, '.mat'));
        map = map.map;
        map.compute_nodepositions();
    elseif exist(fullfile(datadir, strcat(xml_name, '.mat')), 'file')
        fprintf('Pre-parsed file exist, loading ...\n');    
        map = load(fullfile(datadir, strcat(xml_name, '.mat')));
        map = map.map;
        map.compute_nodepositions();
    else
        fprintf('Pre-parsed file does not exist, parsing XML file\n');
        fprintf('This will take a while ...\n');
        
        if ~isempty(getenv("OSM_MAP_PATH")) && ...
            exist(fullfile(getenv("OSM_MAP_PATH"), figurefile), 'file')
            fig_file_path = strcat(getenv("OSM_MAP_PATH"), figurefile);
        elseif exist(figurefile, 'file')
            fig_file_path = figurefile;
        elseif exist(fullfile(datadir, figurefile), 'file')
            fig_file_path = fullfile(datadir, figurefile);
        else
            error("Figure file %s can't be found\n", figurefile);
        end
        
        tic;
        map = OpenStreetMap(fullfile(datadir, xmlfile), fig_file_path);
        fprintf('Done parsing the XML file in %.2f seconds!\n', toc);
        fprintf('Saving data in file %s\n', strcat(xml_name, '.mat'));  
        save(strcat(xml_name, '.mat'), 'map');
    end
end
