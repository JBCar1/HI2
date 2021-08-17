classdef FIFO < handle
    properties (SetAccess = private)               
        v;        
    end
    methods (Access=public)
        function obj=FIFO()
            % FIFO queue object
            obj.v = [];
        end
        
        function insert(obj, x)
            % INSERT  Insert object x in queue
            %
            % Example: 
            %   q = FIFO()
            %   q.insert(10)            
           obj.v = [obj.v; x]; 
        end
        
        function x = pop(obj)            
            % POP  Pop object x from queue
            %
            % Example: 
            %   q = FIFO()
            %   q.insert(10)
            %   q.pop()
            if obj.isempty()
                error( 'called pop() on an empty priority queue' );
            end          
           x = obj.v(1, :);
           obj.v = obj.v(2:end, :);
        end
        
        function r = isempty(obj)
            % ISEMPTY  Return true if queue is empty
            %
            % Example: 
            %   q = FIFO()
            %   q.insert(10)
            %   q.isempty()
            r = size(obj.v, 1) == 0;
        end
        
        function s = size(obj)
            % SIZE  Return number of elements in queue
            %
            % Example: 
            %   q = FIFO()
            %   q.insert(10)
            %   q.size()
            s = size(obj.v, 1);
        end
        
        function x = peek(obj)
            % PEEK  Return next object to pop, without removing the object
            %       from the queue
            %
            % Example: 
            %   q = FIFO()
            %   q.insert(10)
            %   q.peek()
            x = obj.v(1, :);
        end
    end
end