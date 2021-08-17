# Python installation instructions
The python versions of the handins require a fairly recent Python installation (>=3.7 should be fine).

In the student labs, there is a pre-prepared virtual environment with all necessary packages installed. You activate the virtual environment as
```
% source /courses/tsfs12/env/bin/activate
```
If you install at home, we recommend to create your own virtual environment for the handins as
```
% python3 -m venv env
% source env/bin/activate  # On Linux or Mac
% env\Scripts\activate  # On Windows
(env) % pip install -U pip  # Always a good idea to update the package installer
```
and then install all required packages with the command
```
(env) % pip install -r requirements.txt
``` 
You can find the file ```requirements.txt``` in the ```Handin_Exercises``` folder of this git repository.
