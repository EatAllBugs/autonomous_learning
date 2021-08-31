// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Actor/CarlaActor.h"
#include "Carla/Sensor/DataStream.h"

#include "CoreMinimal.h"

class UCarlaEpisode;

class FCarlaServer
{
public:

  FCarlaServer();

  ~FCarlaServer();

  FDataMultiStream Start(uint16_t RPCPort, uint16_t StreamingPort);

  void NotifyBeginEpisode(UCarlaEpisode &Episode);

  void NotifyEndEpisode();

  void AsyncRun(uint32 NumberOfWorkerThreads);

  void RunSome(uint32 Milliseconds);

  bool TickCueReceived();

  void Stop();

  FDataStream OpenStream() const;

private:

  class FPimpl;
  TUniquePtr<FPimpl> Pimpl;
};
