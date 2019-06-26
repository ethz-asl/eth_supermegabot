/*!
 * @file	  ControllerSwapStateInterface.hpp
 * @author	Gabriel Hottiger
 * @date	  Mar 10, 2017
 */

#pragma once

// STL
#include <iostream>
#include <memory>

namespace roco {

class ControllerSwapStateInterface
{
 public:
  //! Constructor
  ControllerSwapStateInterface() { }

  //! Destructor
  virtual ~ControllerSwapStateInterface() { }

  /** Overloads the == operator, to compare two states.
   *  ( Could be used for testing consistency of state before actual switch )
   *  @param  state,  swap state to compare with *this
   *  @return true,   iff states are equal
   */
  virtual bool operator== ( const ControllerSwapStateInterface& state ) const = 0;

  /** Overloads the != operator, to compare two states. Defualts to !operator ==()
   *  ( Could be used for testing consistency of state before actual switch )
   *  @param  state,  swap state to compare with *this
   *  @return true,   iff states are not equal
   */
  virtual bool operator!= ( const ControllerSwapStateInterface& state ) const {
    return !( operator==(state) );
  }

  /** Overloads to << ostream operator for swap states, this allows e.g. std::cout << mystate << std::endl
   * @param out ostream of the << operation
   * @return ostream
   */
  friend std::ostream& operator << (std::ostream& out, const ControllerSwapStateInterface& state)
  {
    state.print(out);
      return out;
  }

protected:
  /** Virtual print helper method for polymorph << ostream operator overload
   *  @param out ostream of the << operation
   */
  virtual void print( std::ostream& out) const { };
};

using ControllerSwapStateInterfacePtr = std::unique_ptr<ControllerSwapStateInterface>;

} /* namespace roco */

