/** \file Signal.hpp
* \brief Header file providing the Signal class interface
*/

#pragma once

#include <map>
#include <list>
#include <csignal>
#include <functional>
#include <mutex>

namespace signal_handler {
/** \brief Signal handling.
*
* This class provides a static interface to bind a common process signal handler.
*/

class SignalHandler {
public:
  SignalHandler() = delete;

  using Handler = std::function<void(int)>;

  template <typename T> static void bind(int signal, void(T::*fp)(int), T* object) {
      SignalHandler::bind(signal, std::bind(fp, object, std::placeholders::_1));
  }

  static void bind(int signal, const Handler& handler);

  template <typename T> static void bindAll(void(T::*fp)(int), T* object) {
      const Handler handler = std::bind(fp, object, std::placeholders::_1);
      SignalHandler::bind(SIGINT, handler);
      SignalHandler::bind(SIGTERM, handler); // shell command kill
      SignalHandler::bind(SIGABRT, handler); // invoked by abort();
      SignalHandler::bind(SIGFPE, handler);
      SignalHandler::bind(SIGILL, handler);
      SignalHandler::bind(SIGQUIT, handler); // the QUIT character, usually C-'\'
      SignalHandler::bind(SIGHUP, handler);  // hang-up” signal is used to report that the user’s terminal is disconnected
      // SIGKILL cannot be handled
  }

  template <typename T> static void unbind(int signal, void(T::*fp)(int), T* object) {
      SignalHandler::unbind(signal, std::bind(fp, object, std::placeholders::_1));
  }

  static void unbind(int signal, const Handler& handler);

  template <typename T> static void unbindAll(void(T::*fp)(int), T* object) {
      const Handler handler = std::bind(fp, object, std::placeholders::_1);
      SignalHandler::unbind(SIGINT, handler);
      SignalHandler::unbind(SIGTERM, handler);
      SignalHandler::unbind(SIGABRT, handler);
      SignalHandler::unbind(SIGFPE, handler);
      SignalHandler::unbind(SIGILL, handler);
      SignalHandler::unbind(SIGQUIT, handler);
      SignalHandler::unbind(SIGHUP, handler);
  }

private:
  static std::map<int, std::list<Handler> > handlers;
  static std::mutex mutex;

  static void signaled(int signal);
};

} // namespace signal_handler
