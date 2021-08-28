classdef PriorityQueue < handle
  properties (SetAccess = private)               
   numElements; 
   priorityList;
   valueList;
   flagMaxPriorityQueue;
  end

  methods (Access = public)
      function obj = PriorityQueue(flagMaxPriorityQueue)
        % Create an empty priority queue
        %
        % Priority queue implementation borrowed from 
        % https://stackoverflow.com/questions/12469502/how-to-implement-priority-queue-in-matlab
        if ~exist( 'flagMaxPriorityQueue', 'var' )
          flagMaxPriorityQueue = false;
        else
          if ~(isscalar(flagMaxPriorityQueue) && islogical(flagMaxPriorityQueue))
            error( 'ERROR: invalid flagMaxPriorityQueue argument' );
          end
        end

        obj.flagMaxPriorityQueue = flagMaxPriorityQueue;
        obj.numElements = 0;
        obj.priorityList = {};
        obj.valueList = {};
      end

      function insert(obj, value, priority)
        % INSERT an element in the queue
        %
        % obj.INSERT(value, priority)
        %
        % Example: 
        %   q = PriorityQueue()
        %   q.insert('a', 10)
        %   q.insert('b', 5)
        
        % increase the size of the array if full
        if obj.numElements > 0 && obj.numElements + 1 > numel( obj.priorityList )                                
          % double the size of the array and copy stuff
          obj.priorityList = cat(1, obj.priorityList, cell(obj.numElements, 1));
          obj.valueList = cat(1, obj.valueList, cell(obj.numElements, 1));
        end
        obj.numElements = obj.numElements + 1;

        obj.priorityList{ obj.numElements } = priority;
        obj.valueList{ obj.numElements } = value;

        obj.swim(obj.numElements);
      end

      function [value, priority] = pop(obj)
        % POP the element with lowest priority from the queue
        %
        % [value, priority] = obj.POP()
        %
        % Example: 
        %   q = PriorityQueue()
        %   q.insert(10, 'a')
        %   q.insert(5, 'b')
        %   [x, prio] = q.pop();
        if obj.isempty()
          error( 'called pop() on an empty priority queue' );
        end          
        priority = obj.priorityList{1};
        value = obj.valueList{1};

        obj.exch(1, obj.numElements);            
        obj.numElements = obj.numElements - 1;            
        obj.sink(1);

        obj.priorityList{ obj.numElements + 1 } = [];
        obj.valueList{ obj.numElements + 1 } = [];

        % halve the size of the arrays if they get one-quarter full
        if obj.numElements > 0 && obj.numElements == floor( numel( obj.priorityList ) / 4 )                
          obj.priorityList( 2 * obj.numElements + 1 : end ) = [];
          obj.valueList( 2 * obj.numElements + 1 : end ) = [];
        end
      end

      function [flagEmpty] = isempty(obj)
        % ISEMPTY Check if queue is empty
        % ISEMPTY  Return true if queue is empty
        %
        % Example: 
        %   q = PriorityQueue()
        %   q.insert(10, 'a')
        %   q.isempty()
        
        flagEmpty = (obj.numElements == 0);
      end

      function [qSize] = size(obj)
        % SIZE  Return number of elements in queue
        %
        % Example: 
        %   q = PriorityQueue()
        %   q.insert(10, 'a')
        %   q.size()
        qSize = obj.numElements;
      end

      function [value, priority] = peek(obj)
        % PEEK Peek into the queue without removing element
        %
        % [value, priority] = obj.peek()
        %
        % Example: 
        %   q = PriorityQueue()
        %   q.insert(10, 'a')
        %   q.insert(5, 'b')
        %   [prio, x] = q.peek();
        if obj.isempty()
          error( 'requested max() of an empty priority queue' );
        end
        priority = obj.priorityList{1};
        value = obj.valueList{1};
      end
  end    

  methods (Access = private)
    function swim(obj, elPos)
      while elPos > 1 && obj.compare(floor(elPos / 2), elPos)
          obj.exch(floor(elPos / 2), elPos);
          elPos = floor(elPos / 2);
      end
    end

    function sink(obj, elPos)
      while 2 * elPos <= obj.numElements
        j = 2 * elPos;
        if j < obj.numElements && obj.compare(j, j+1)
          j = j + 1;
        end
        if ~obj.compare(elPos, j)
          break;
        end
        obj.exch(elPos, j);
        elPos = j;
      end
    end

    function [blnCmpResult] = compare(obj, e1, e2)
      if obj.flagMaxPriorityQueue
        blnCmpResult = (obj.priorityList{e1} < obj.priorityList{e2});
      else
        blnCmpResult = (obj.priorityList{e1} > obj.priorityList{e2});
      end            
    end

    function exch(obj, e1, e2 )
      temp = obj.priorityList{e1};
      obj.priorityList{e1} = obj.priorityList{e2};
      obj.priorityList{e2} = temp;            

      temp = obj.valueList{e1};
      obj.valueList{e1} = obj.valueList{e2};
      obj.valueList{e2} = temp;            
    end
  end
end % classdef