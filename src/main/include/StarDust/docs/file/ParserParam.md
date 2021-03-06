# ParserParam

```cpp
#include "StarDust/file/ParserParam.hpp"
```

The `ParserParam` class is NOT a `StarDustComponent`, it should not be passed into a `StarDustRobot`.

`ParserParam` objects allow for the `ConfigParser` to know what things to parse.

## Initialization

#### `template<typename T> ParserParam(std::string param, T* variable);`

Creates a new parameter that parses variables named `name`, and references the variable to update via `var`.

Type `T` can be any of the following:

```
int
float
double
bool
std::string

std::vector<T> //where T is one of the above types
```

Initialization example:

```cpp
int var1=123;
double var2=3.14;
std::vector<std::string> var3={ "default", "data" };

ParserParam<int> param1 { "var1", &var1 };
ParserParam<double> param2 { "var2", &var2 };
ParserParam<std::vector<std::string>> param3 { "var3", &var3 };
```

If parsing of a variable fails, the default value for type `T` is used.

#### `template<typename T> ParserParam(std::string name, T* var, T iffail);`

Same as above, but `iffail` is used instead of default for type `T`.

Initialization example:

```cpp
int var=123
ParserParam param { "var", &var, -1 };
```

## Functions

#### `template<typename T> void parse(std::string data, T* input, T fail)`

This function will (try to) parse string data and update the original value with the new data.

To see syntax for variable types, look at the [ConfigParser](/docs/file/ConfigParser.md) docs.

#### `bool convert(std::string name, std::string data)`

Returns true if passed in `name` matches parameter name.

This function is only really meant to be called by the `ConfigParser`.

## Variables

#### `std::string param;`

Stores the name of the the variable to be parsed.

#### `T* variable;`

Reference to the variable to be updated after parsing.

#### `T fail;`

Value to use if parsing fails.`