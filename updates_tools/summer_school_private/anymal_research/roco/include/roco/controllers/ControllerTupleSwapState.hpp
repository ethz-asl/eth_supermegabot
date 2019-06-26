/*!
 * @file	  ControllerTupleSwapState.hpp
 * @author	Gabriel Hottiger
 * @date	  Mar 10, 2017
 */

#pragma once

// roco
#include "roco/controllers/ControllerSwapStateInterface.hpp"

// STL
#include <vector>
#include <memory>

namespace roco {

class ControllerTupleSwapState : public ControllerSwapStateInterface
{
 public:
  /** Constructor
   *  @param nrStates number of states/controllers
   */
  ControllerTupleSwapState(const std::size_t nrStates)
   : states_(nrStates)
  {

  }

  //! Delete default constructor
  ControllerTupleSwapState() = delete;

  //! Destructor
  virtual ~ControllerTupleSwapState() { }

  /** Overloads the == operator, to compare two states. Returns true by default!
   *  ( Could be used for testing consistency of state before actual switch )
   *  @param  state,  swap state to compare with *this
   *  @return true,   iff states are equal
   */
  virtual bool operator== ( const ControllerSwapStateInterface& state ) const {
    for(auto & stateEntry : states_ ) { if( !(*stateEntry == state) ) { return false; } }
    return true;
  }

  /** Overloads the != operator, to compare two states. Returns true by default!
   *  ( Could be used for testing consistency of state before actual switch )
   *  @param  state,  swap state to compare with *this
   *  @return true,   iff states are not equal
   */
  virtual bool operator!= ( const ControllerSwapStateInterface& state ) const {
    for(auto & stateEntry : states_ ) { if( !(*stateEntry != state) ) { return false; } }
    return true;
  }

  std::unique_ptr<ControllerSwapStateInterface> & getSwapState(const unsigned int i) {
    return states_.at(i);
  }

  const std::vector< std::unique_ptr<ControllerSwapStateInterface> > & getSwapStates() const {
    return states_;
  }

  const std::size_t size() const { return states_.size(); }

 protected:
  std::vector< std::unique_ptr<ControllerSwapStateInterface> > states_;

};

} /* namespace roco */

