/********************************************************
  Stanford Driving Software
  Copyright (c) 2011 Stanford University
  All rights reserved.

  Redistribution and use in source and binary forms, with 
  or without modification, are permitted provided that the 
  following conditions are met:

* Redistributions of source code must retain the above 
  copyright notice, this list of conditions and the 
  following disclaimer.
* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the 
  following disclaimer in the documentation and/or other
  materials provided with the distribution.
* The names of the contributors may not be used to endorse
  or promote products derived from this software
  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGE.
 ********************************************************/


#include <aw_StWaitForActivation.hpp>
#include <aw_StError.hpp>

namespace vlr {

StError::StError(my_context ctx) :
  my_base(ctx), StBase<StError>(std::string("StError")) {
  // TODO: recreate as much as possible: topology, vehiclemananger, routesampler - or restart?
}

StError::~StError() {
}

sc::result StError::react(const EvProcess&) {
  if (detectedErrornousTransitions()) std::cout << "oooops" << std::endl;

  ChsmPlanner& planner = context<ChsmPlanner> ();
  // set velocity to zero
  planner.generateStopTrajectory();
  planner.velocity_desired_ = 0; // park mode
//  planner.vehiclecmd.beeper_on = 1;
//  planner.vehiclecmd.hazard_lights_on = 1;
//  planner.vehiclecmd.warninglights_on = 1; // indicate error
  return forward_event();
}

} // namespace vlr
