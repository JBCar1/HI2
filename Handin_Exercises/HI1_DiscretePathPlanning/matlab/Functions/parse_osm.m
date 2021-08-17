function res = parse_osm(tree)
  children = tree.getChildNodes();
  numChildren = children.getLength();
  for k=1:numChildren
    child = children.item(k-1);
    nodeName = char(child.getNodeName());
    if strcmp(nodeName, 'osm') 
      res = parse_osm_tree(child);
    end
  end
end

function res = parse_osm_tree(osm)
  children = osm.getChildNodes();
  numChildren = children.getLength();

  res.bounds = [];
  res.nodes = containers.Map('KeyType', 'int64', 'ValueType', 'any');
  res.ways = {};
  for k=1:numChildren
    child = children.item(k-1);
    nodeName = char(child.getNodeName());
    if strcmp(nodeName, 'bounds') 
      res.bounds = parse_osm_bounds(child);
    elseif strcmp(nodeName, 'node')
      nodeInfo = parse_osm_node(child);
      res.nodes(nodeInfo.id) = nodeInfo.xy;
    elseif strcmp(nodeName, 'way')
      [wayNode, highway] = parse_osm_way(child);
      if highway
        res.ways{end+1} = wayNode;
      end
    end
  end
end

function res = parse_osm_bounds(node)
  res = [];
  if node.hasAttributes && node.getAttributes().getLength()==4
    att = node.getAttributes();
    for k=0:3
      att_k = att.item(k);
      name = char(att_k.getName());
      val = str2double(char(att_k.getValue()));
      res.(name) = val;
    end
  end
end

function res = parse_osm_node(node)
  res = [];
  nodeInfo.id = 0;
  nodeInfo.lat = 0;
  nodeInfo.lon = 0;
  if node.hasAttributes
    att = node.getAttributes();
    nAtt = att.getLength();
    for k=0:nAtt-1
      att_k = att.item(k);
      name = char(att_k.getName());
      switch name
        case {'id', 'lat', 'lon'}
          val = str2double(char(att_k.getValue()));
          nodeInfo.(name) = val;
      end
    end
  end
  res.id = nodeInfo.id;
  res.xy = [nodeInfo.lon, nodeInfo.lat];
end

function [res, carRoad] = parse_osm_way(node)
  res.id = 0;
  res.nodes = [];
  res.maxspeed = -1;
  res.lanes = -1;
  res.name = '';
  carRoad = false;
  
  road_vals = {'motorway', 'motorway_link', 'trunk', 'trunk_link',...
             'primary', 'primary_link', 'secondary', 'secondary_link',...
             'tertiary', 'road', 'residential', 'living_street',...
             'service', 'services', 'motorway_junction', 'unclassified'};

  if node.hasAttributes
    att = node.getAttributes();
    nAtt = att.getLength();
    for k=0:nAtt-1
      att_k = att.item(k);
      name = char(att_k.getName());
      switch name
        case 'id'
          val = str2double(char(att_k.getValue()));
          res.(name) = val;
      end
    end
    
    children = node.getChildNodes();
    numChildren = children.getLength();
    for k=0:numChildren-1
      child = children.item(k);
      name = char(child.getNodeName());
      switch name
        case 'nd'
          att = child.getAttributes();
          if att.getLength()==1 && strcmp(att.item(0).getName(),'ref')
            node_id = str2double(char(att.item(0).getValue()));
            res.nodes(end+1) = node_id;
          end
        case 'tag'
          tag.k = '';
          tag.v = '';
          att = child.getAttributes();
          for l=0:att.getLength()-1
            if strcmp(att.item(l).getName(),'k')
              tag.k = char(att.item(l).getValue());
            elseif strcmp(att.item(l).getName(),'v')
              tag.v = char(att.item(l).getValue());
            end
          end
          switch tag.k
            case {'lanes', 'maxspeed'}
              res.(tag.k) = str2double(tag.v);
            case 'name'
              res.name = tag.v;            
            case 'highway'
              if ismember(tag.v, road_vals)
                % disp(tag.v);
                carRoad = true;
              end
          end
      end
    end
  end
end
