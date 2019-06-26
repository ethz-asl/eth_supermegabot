/*! \page page_time Time
<H3>Collect Time</H3>
The time is automatically logged by the signal logger at every call of collectLoggerData(). For space-efficiency reasons the time is only recorded once for
all the logger elements. When further processing a log element (e.g. publishing, saving), the timestamps are matched to the data points of the log element.

Time is stored as an epoch time in seconds and nanoseconds. The std logger uses the system clock as timestamp generator, the ros logger uses ros::Time::now() in
order to support simulation time.

<H3>Time Buffer</H3>
<H4>Case 1: 'maxLogTime != 0'</H4>
A fixed size buffer (signal_logger::BufferType::FIXED_SIZE)
of length maxLogTime*collectFrequency is allocated. Once the buffer is full, logging is automatically stopped.

<H4>Case 2: 'maxLogTime == 0' (or unspecified, defaults to 0)</H4>
The time buffer is set to type signal_logger::BufferType::EXPONENTIALLY_GROWING with an initial length of 10*collectFrequency
(thus allows recording 10s without reallocation), when the buffer is full the buffer size is increased by a factor of two.<BR>
<B>Increasing buffer size (reallocating) is very time inefficient, in time critical application do not use this buffer type!</B>

<H4>Case 3: 'bufferType == signal_logger::BufferType::LOOPING' for all elements</H4>
If the all log elements are of signal_logger::BufferType::LOOPING at logger start, then automatically a looping buffer is used for the time.
This allows continuous logging and publishing (e.g via ros). The buffer size is calculated by max(divider*bufferSize) of all elements that are currently active.
*/
