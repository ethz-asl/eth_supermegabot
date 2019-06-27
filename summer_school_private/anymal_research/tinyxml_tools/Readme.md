# tinyxml_tools

## Overview

Parse and load paremeters from XML files.

Wraps around the TinyXML library.

## Usage
The most basic usage is
```
DocumentXMLHandler docHandler;
docHandler.create("path/to/my/xml.xml", tinyxml_tools::DocumentMode::<Your mode here>);
```
`path/to/my/xml.xml`can be a hard coded address (ie `/home/username/path/to/the/xml.xml`) or a relative address to the pwd (ie `path/to/my/xml.xml`).
If the file does not exist but a writable mode was chosen, `DocumentXMLHandler::create(...)` will return an empty xml file (which is a file with the necessary xml headers).
Reading and writing the xml can be done via either DocumentXMLHandler object or by calling `DocumentXMLHandler::getRootHandle()` or `DocumentXMLHandler::getDocumentHandle()`, which return the underlying TiXmlHandle objects used to access the file (documentation for TinyXML can be found [here](http://www.grinninglizard.com/tinyxmldocs/))

### Additions
There are a few non-tinyxml utilities that TinyXML_tools provides.
#### Default
In this file, default param values are given.
```
<?xml version="1.0" ?>
<Root default="default.xml" >
    <CustomDefault value="0"/>
</Root>
```
The corresponding default.xml file might look like
```
<?xml version="1.0" ?>
<Root>
    <DefaultOnly value="1"/>
    <CustomDefault value="1"/>
</Root>
```
All values in the non-default file must exist in  default file. The values in the non-default file overwrite those in the default file, while values in the default file but not in the non-default file will stay as the default. However, values in the non-default file that do not exist in the default file cause the load to fail, and may generate a xml file with unexpected values. Calling `DocumentXMLHandler::saveAs()` after loading the non-default file yields
```
<?xml version="1.0" ?>
<Root>
    <CustomDefault value="0" />
    <DefaultOnly value="1" />
</Root>

```

#### Extension
In this file, default param values are given.
```
<?xml version="1.0" ?>
<Root extension="extension.xml">
    <CustomOnly value="0"/>
    <CustomExtension value="0"/>
</Root>
```
The corresponding extension.xml file might look like
```
<?xml version="1.0" ?>
<Root>
    <ExtensionOnly value="2"/>
    <CustomExtension value="2"/>
</Root>
```
The values in the non-extension file overwrite those in the extesion file, while values in the extension file but not in the non-extension file will stay as the extension value. Unlike Default, values in the custom that are not in the extension file are preserved. Calling `DocumentXMLHandler::saveAs()` after loading the non-extension file yields
```
<?xml version="1.0" ?>
<Root>
    <CustomOnly value="0" />
    <CustomExtension value="0" />
    <ExtensionOnly value="2" />
</Root>
```

#### Extension and Default Together

Default and Extension are designed to be used together. Here they are (this file is henceforth referred to as the custom file):
```
<?xml version="1.0" ?>
<Root default="default.xml" extension="extension.xml">
    <CustomDefault value="0"/>
    <CustomDefaultExtension value="0"/>
</Root>
```
The default.xml looks like
```
<?xml version="1.0" ?>
<Root>
    <DefaultOnly value="1"/>
    <CustomDefault value="1"/>
    <DefaultExtension value="1"/>
    <CustomDefaultExtension value="1"/>
</Root>
```
And the extension.xml looks like
```
<?xml version="1.0" ?>
<Root>
    <ExtensionOnly value="2"/>
    <DefaultExtension value="2"/>
    <CustomDefaultExtension value="2"/>
</Root>
```
Here, values in the custom file take priority over extension or default values. Default values take precedent over extension values, but extension is allowed to have values that are not found in the default file. Calling `DocumentXMLHandler::saveAs()` after loading the non-extension file yields
```
<?xml version="1.0" ?>
<Root>
    <DefaultOnly value="1" />
    <CustomDefault value="0" />
    <DefaultExtension value="1" />
    <CustomDefaultExtension value="0" />
    <ExtensionOnly value="2" />
</Root>
```

#### The include_xml Tag
Within an xml file, you can add include tags.
```
<?xml version="1.0" ?>
<?xml version="1.0" ?>
<Root>
    <TopLevel value="1" />
    <IncludedNode>
      <include_xml dir="include_xml/include.xml" />
    </IncludedNode>
    <IncludedWithoutRoot>
      <include_xml dir="include_xml/include_ignore_root.xml" />
    </IncludedWithoutRoot>
</Root>

```
include.xml:
```
<?xml version="1.0" ?>
<Included>
    <IncludedValue value="1" />
</Included>
```
include_ignore_root:
```
<?xml version="1.0" ?>
<ignore_root>
    <IncludedValue value="1" />
</ignore_root>
```
The directory structure is
```
.
├── custom.xml
└── include_xml
    ├── include_ignore_root.xml
    └── include.xml
```
The include_xml tag lets you include xml files specifed by the value given to `dir` into your current xml file. They are included into the xml structure under the tags where the `include_xml` call is made. The tag `<ignore_root>` causes the include to discard the root of the included file. Calling `DocumentXMLHandler::saveAs()` after loading custom.xml yields
```
<?xml version="1.0" ?>
<Root>
    <TopLevel value="1" />
    <IncludedNode>
        <Included>
            <IncludedValue value="1" />
        </Included>
    </IncludedNode>
    <IncludedWithoutRoot>
        <IncludedValue value="1" />
    </IncludedWithoutRoot>
</Root>
```
The paths given to `dir` can be relative paths to the xml file, hard coded paths (ie `/home/username/path/to/include/xml.xml`), or relative to the pwd by using `./path/relatvie/to/pwd.xml`. Importantly, `./*` does NOT mean relative to the xml file path.