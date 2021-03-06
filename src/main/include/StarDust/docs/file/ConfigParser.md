# ConfigParser

```cpp
#include "StarDust/file/ConfigParser.hpp"
```

`ConfigParser` is a flexible config file parser.

## Initialization

#### `ConfigParser(std::vector<ParserParamBase*> parameters)`

Ceation of a `ConfigParser` requires passing in many `ParserParam`s:

```cpp
//example variables
double multiplier=0;
std::string name="";

ConfigParser parser {{
    new ParserParam { "multiplier", &multiplier },
    new ParserParam { "name", &name }
}};
```

Now, the parser will be able to parse and override the `name` and `multiplier` variables.

By default, the config file will live at `"/home/lvuser/config.dat"` on the robo-rio.

#### `ConfigParser(std::vector<ParserParamBase*> parameters, std::string filename)`

Same as before, but a different filename can be passed in:

```cpp
ConfigParser parser {{
    //...
}, "/path/to/file/on/robo-rio"};
```

`ConfigParser` will now look for a file at `name` instead.

## Usage

One of the reasons to use a config file parser is to reduce the time you spend re-compiling code.

With `ConfigParser`, you can parse all of the following variables:

```
int
float
double
bool
std::string

std::vector<T> //where T is one of the above types
```

Here is an example config file explaining syntax and usage:

```
//name is on the left of the equal sign, value on the right
value=0

//number values can have leading or trailing spaces:
value= 0
value=0   < spaces end here

//but variable names must be exact
//all of these are invalid
value =0
 value=0
 value =0

//comments are any code that isnt a variable name
this is a valid comment (although not reccommended)

//comments must be on their own line, below will not work
value=0 //comment

//string variables are treated as raw strings
//for example:
name= bob

//name is now " bob", not "bob"

//bools are parsed like strings, the following is not allowed:
test= false
test=true  < spaces

//vectors must have an opening and closing []
ints=[1,2,3]

//vectors of numbers can have spaces
ints=[ 1, 2, 3 ]

//string and bool vectors are still treated like raw strings:
strings=[ hello, world! ]

//strings contains [" hello", " world! "] now
```

## RobotFunctions

Below is a list of what this component does during certain robot functions.

```
//these run the autorun() function
__RobotInit__()
__AutonomousInit__()
__TeleopInit__()
```

## Functions

#### `void autorun();`

This function will automatically find and parse the config file.

## Variables

#### `std::string filename="/home/lvuser/config.dat";`

Stores the path of the config file to be parsed.

#### `std::vector<ParserParamBase*> parameters;`

Stores the parameters passed in from the construction of the `ConfigParser`.