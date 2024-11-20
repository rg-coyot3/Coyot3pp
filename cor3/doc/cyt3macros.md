# COYOT3PP::COR3 CYT3MACROS



## C++ Preprocessor macros/tools : `CYT3MACRO`s

The `CYT3MACRO`s are a good example of what we can do when using C++ preprocessor macros using the advantages given since c++20 to be able to expand variadic arguments, when we have very clear ideas of the code we want to "print".

### ENUM CLASS
    
Creates an enum class with stringification ad de-stringification function/methods. If the enum class is declared as standalone (not inside of another class), then it also defines some operator overloads:

* It will automatically add 2 values: INTERNAL_ERROR = -2, and UNKNOWN_UNSET = -1
* `::std::ostream& operator<<(::std::ostream& o,const CY_enum_class_name& s);)`
  Makes easy the generation of logs to follow a state.
* `int& operator<<(int& o,CY_enum_class_name s)` : to port as integers
* `CY_enum_class_name& operator<<(CY_enum_class_name& o,int s)` : to port from integers.
* `CY_enum_class_name& operator<<(CY_enum_class_name& o, const std::string& i);` to deserialize
* Other functions:
  * `static std::string <enum class name>ToString(<enum class name> v)`
  * `static <enum class name> <enum class name>FromString(const std::string& v)`


**SYNTAX**

```cpp
CYT3MACRO_enum_class_declarations(
  name-of-the-enum-class
  , name-of-the-container-class not required
  , ... sequence of items with values if needed
)

    CYT3MACRO_enum_class_definitions(
(
  name-of-the-enum-class
  , name-of-the-container-class not required
  , ... sequence of items without the values...
)


EXAMPLE:
CYT3MACRO_enum_class_declarations(
  ExampleEnumClass
  ,
  , ONE_VALUE = 0
  , OTHER_VALUE = 1
  , ANOTHER_VALUE = 2
)

CYT3MACRO_enum_class_definitions(
  ExampleEnumClass
  ,
  , ONE_VALUE
  , OTHER_VALUE
  , ANOTHER_VALUE
)

ec::ExampleEnumClass example;
example = ec::ExampleEnumClass::ONE_VALUE;
std::cout << "example = " << example;
```



### MODEL CLASS
    
The goal of this macro is to create data oriented classes, make some standard operations and be able to easily maintain them along the lifecycle of a project.

**SYNTAX**

Both `_declarations` and `_definitions` have the same format.

```cpp
CYT3MACRO_model_class_<declarations | definitions>(
  NameOfTheModelClass
  , ParentModelClass
  , ( variadic : additional declarations to be defined at .cpp)
  , ( enum class dependences # to be deprecated )
  
    sequence of triples:
    , name of the property      , type of property    , default-value
)

```

* Each property will be accessible as a method. Has the following overloads:
  * `const property_type& property() const` : getter
  * `property_type& property();` setter
  * `property_type& property(const property_type& value);` setter
* The class contains operation overloads:
  * `operator=(const ClassName& o)`
  * `operator==(const ClassName& o)`
* It also includes a static method "explaning" the class:
  * `static std::string get_model_template();`
* Also includes the following constructors:
  * base ; copy ; by parameters


As an example: 

```cpp
CYT3MACRO_model_class_declarations(
  Position
  , 
  , ( virtual std::string to_string() const)
  , ( )
    , latitude      , double      , 0.0
    , longitude     , double      , 0.0
    , altitude      , double      , 0.0
)

CYT3MACRO_model_class_definitions(
  Position
  , 
  , ( virtual std::string to_string() const)
  , ( )
    , latitude      , double      , 0.0
    , longitude     , double      , 0.0
    , altitude      , double      , 0.0
)


std::string Position::to_string() const {
  std::stringstream sstr;
  sstr << "lat:" <<latitude() << ",lon:" << longitude() << ",alt:" << altitude();
  return sstr.str();
}

Position pos(111.1,222.2,333.3);
pos.altitude(123.32);
pos.longitude() = 123;
pos.latitude(pos.longitude());

std::cout << pos.to_string() << std::endl;
std::cout << Position::get_model_template() << std::endl;

```
  

It is possible to extend the class:


```cpp
CYT3MACRO_model_class_declarations(
  PosOrient
  , Position
  , ( virtual std::string to_string() const)
  , ( )
  , orientation   , double      , 0.0
)
CYT3MACRO_model_class_definitions(
  PosOrient
  , Position
  , ( virtual std::string to_string() const)
  , ( )
  , orientation   , double      , 0.0
)

std::string PosOrient::to_string() const {
  return Position::to_string() + std::string(",hea:") + std::to_string(orientation());
}

PosOrient por(pos);
por.orientation(90.5);
std::cout << por.to_string() << std::endl;
std::cout << PosOrient::get_model_template() << std::endl;

PosOrient por2;
std::cout << (por == por2) << std::endl;
std::cout << (por2 = por).to_string() << std::endl;
```


### MODEL CLASS JSON SERIALIZATION



The original model class can be extended to be able to be serializable by json. The base library used for this serializations is `jsoncpp`, and the `JsonSerializablePacketBase`.

**SYNTAX**

```cpp
CYT3MACRO_model_class_json_serializable_<declarations & definitions>(
  Name of the class
  ,master class
  , version string
  , ( 
      property name, "property json tag", JsonSerializable base class
      , <sequence of triples>
  )
  , ( 
    property name, "property json tag" , <enum class>
  )
    , property name, "json tag", type cast io
    , <sequence of triples>
)
```

**EXAMPLE**

```cpp
CYT3MACRO_model_class_json_serializable_declarations(
  Position
  ,
  ,
  , ( )
  , ( )
    , latitude      , "latitude"      , 
    , longitude     , "longitude"     , 
    , altitude      , "altitude"      , 
)

CYT3MACRO_model_class_json_serializable_definitions(
  Position
  ,
  ,
  , ( )
  , ( )
    , latitude      , "latitude"      , 
    , longitude     , "longitude"     , 
    , altitude      , "altitude"      , 
)
PositionJsIO posjs(position);

std::cout << posjs;

Json::Value js = posjs.to_json();
PositionJsIO posjs2;

posjs2.from_json(js);

std::cout << " equals? " << (posjs == posjs2) << std::endl;
```

### MODEL CLASS SET STACK (`std::vector` wrapper)

To be able to have a set of items, I created a wrapper over the *model-class* to be able to create sets of items defined with the `model-class` macro.

`CYT3MACRO_model_class_set_stack_<declarations & definitions>(<name of the class>, (optional) max-number-of-items)`

Basically, it will create a class with the `Stack` suffix that contains the following methods:

* **constructors**: base, copy, list.
* `size()`
* operators overload : 
  * `=`
  * `==`: will consider that they are equal if all the items in left operator are included in right operator, with same stack size.
  * `[]` : wrapper to `at` method to obtain *reference* or *const reference* of the item.
* `push_back` and `push_front` methods to insert items *and* *Stacks*. A `clear()` method;
* `at(int index)` method to obtain a reference OR const reference to the item at any position:
  * *warning: index `MUST BE` inferior to the size of the stack.*
* a `for_each` method to iterate over *references* or *const references* of the items.
  * **thread safe method**
  * the method must receive as parameter `bool( ItemClass)` (ok with lambdas). The `for_each` method returns the number of positive (true) operations at its iteration.
* `resize` method that will change the `max-number-of-items` of the stack. 

**EXAMPLE:**


```cpp
CYT3MACRO_model_class_set_stack_declarations(PosOrient,)
CYT3MACRO_model_class_set_stack_definitions(PosOrient,)

PosOrientStack posstack;
posstack.push_back(pos);
posstack.push_front(posor);

//<see minimal example> hello-world
``` 

The stack can also be serialized to JSON with the macro

`CYT3MACRO_model_class_set_stack_serializable_<declarations & definitions>(name-of-the-class)`

It will create a class with the suffix `<NameOfTheClass>StackJsIO`.


### MODEL CLASS MAPPED STACK (`std::map<,>` wrapper)

It is possible to create a mapped stack of the *model-class* with 

`CYT3MACRO_model_class_set_mapped_<declarations & definitions>(name-of-the-class,primary-key)`

Will create a class with the suffix `MappedSet` with the following methods:

* constructors: base, copy and list
* `operator=` overload
* `operator==` overload
* `operator+` and `operator+=` overload.
* `operator[primary-key-type]` and method `get(.)`.
* `insert(class item)`, `update`, `remove` (by primary key or by position), returning `bool true` on success.
* `size()`
* `for_each(bool(const PosOrient&))` and `for_each(bool(PosOrient))`
    
It also has its json-serializable macro

`CYT3MACRO_model_class_set_mapped_json_serializable_<declarations & definitions>(name-of-the-class, primary-key)`

Will create a class with the suffix `MappedSetJsIO`

*see at hello-world.cpp example.*

