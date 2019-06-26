/*! \page page_log_script Log Script
<H3>File structure</H3>
The log script lists all elements that were added to the logger. It can also define (optional) changes from the default
configuration of the logger element. We use YAML as a file format.

Example:

\code{yaml}
  log_elements:
    - name: /myLoggerNamespace/myGroup1/myDataA
      enabled: true
      divider: 1
      buffer:
        type: 0
        size: 5
      action: 0
    - name: /myLoggerNamespace/myGroup1/myDataB
      enabled: false
      buffer:
        type: 1
        size: 50
      action: 1
    - name: /myLoggerNamespace/myGroup2/myDataC
      enabled: true
      divider: 5
      action: 2
    - name: /myLoggerNamespace/myGroup2/myDataD
      enabled: false
      divider: 10
      buffer:
        type: 1
        size: 100
\endcode

The values for buffer:type: corresponds to the integer value of the signal_logger::BufferType. Same applies to action: and signal_logger::LogElementAction.
*/
