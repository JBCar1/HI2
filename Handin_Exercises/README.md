# Python installation instructions

In the student labs, there is a pre-prepared virtual environment with all necessary packages installed. You activate the virtual environment as
```
% source /courses/tsfs12/env/bin/activate
```
If you install at home, we recommend to create your own virtual environment for the handins. First, open a terminal and ensure you have Python installed
```
% python --version  # Name of the binary may vary between installations
```
to verify that you have an up-to-date installation. The handins require >=3.8 should be fine.

Now, create a virtual environment as
```
% python -m venv env
% source env/bin/activate  # On Linux or Mac
% env\Scripts\activate  # On Windows
(env) % pip install -U pip  # Always a good idea to update the package installer
```
and then install all required packages with the command
```
(env) % pip install -r requirements.txt
``` 
You can find the file ```requirements.txt``` in the ```Handin_Exercises``` folder of this git repository.
