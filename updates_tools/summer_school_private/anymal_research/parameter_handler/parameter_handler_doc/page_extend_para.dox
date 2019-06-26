/*! \page page_extend_para Extend the parameter handler with additional types
<H3> Add a new type to the parameter handler </H3>
<ul>
<li> Add traits for your new type to ParameterValueTraits.hpp</li><BR>
NOTE: Even though min/max values are defined, they don't have to be used
in the traits.<BR> E.g. Strings could have min=max="" the value would not be checked
against it.
<li> Add a new services to ParameterHandlerRos.hpp that allow setting/getting of the parameter type</li>
<li> Add these services in the rqt_plugin and define your custom visualization type, deriving from ParameterBase.hpp</li>
</ul>

<H3> Add a new Eigen Matrix type to the parameter handler </H3>
The parameter handler basically supports all Eigen matrix types. However, if you
want to use matrices in the rqt_gui or publish them over the ParameterHandlerRos, you
have to add the matrix type to type_macros.hpp. Only the matrix types of supported scalar
types can be added!

*/
