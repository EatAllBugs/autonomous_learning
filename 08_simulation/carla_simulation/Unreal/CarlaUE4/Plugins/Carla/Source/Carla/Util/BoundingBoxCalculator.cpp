// Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla.h"
#include "Carla/Util/BoundingBoxCalculator.h"

#include "Carla/Traffic/TrafficSignBase.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"

#include "Components/CapsuleComponent.h"
#include "GameFramework/Character.h"

#include "Rendering/SkeletalMeshRenderData.h"
#include "Engine/SkeletalMeshSocket.h"

namespace crp = carla::rpc;

static FBoundingBox ApplyTransformToBB(
    const FBoundingBox& InBoundingBox,
    const FTransform& Transform)
{
  const FRotator Rotation = Transform.GetRotation().Rotator();
  const FVector Translation = Transform.GetLocation();
  const FVector Scale = Transform.GetScale3D();

  FBoundingBox BoundingBox = InBoundingBox;
  BoundingBox.Origin *= Scale;
  BoundingBox.Origin = Rotation.RotateVector(BoundingBox.Origin) + Translation;
  BoundingBox.Extent *= Scale;
  BoundingBox.Rotation = Rotation;

  return BoundingBox;
}

FBoundingBox UBoundingBoxCalculator::GetActorBoundingBox(const AActor *Actor, uint8 InTagQueried)
{
  if (Actor != nullptr)
  {
    // Vehicle.
    auto Vehicle = Cast<ACarlaWheeledVehicle>(Actor);
    if (Vehicle != nullptr)
    {
      FVector Origin = Vehicle->GetVehicleBoundingBoxTransform().GetTranslation();
      FVector Extent = Vehicle->GetVehicleBoundingBoxExtent();
      return {Origin, Extent};
    }
    // Walker.
    auto Character = Cast<ACharacter>(Actor);
    if (Character != nullptr)
    {
      auto Capsule = Character->GetCapsuleComponent();
      if (Capsule != nullptr)
      {
        const auto Radius = Capsule->GetScaledCapsuleRadius();
        const auto HalfHeight = Capsule->GetScaledCapsuleHalfHeight();
        // Characters have the pivot point centered.
        FVector Origin = {0.0f, 0.0f, 0.0f};
        FVector Extent = {Radius, Radius, HalfHeight};
        return {Origin, Extent};
      }
    }
    // Traffic sign.
    auto TrafficSign = Cast<ATrafficSignBase>(Actor);
    if (TrafficSign != nullptr)
    {
      auto TriggerVolume = TrafficSign->GetTriggerVolume();
      if (TriggerVolume != nullptr)
      {
        FVector Origin = TriggerVolume->GetRelativeTransform().GetTranslation();
        FVector Extent = TriggerVolume->GetScaledBoxExtent();
        return {Origin, Extent};
      }
      else
      {
        UE_LOG(LogCarla, Warning, TEXT("Traffic sign missing trigger volume: %s"), *Actor->GetName());
        return {};
      }
    }
    // Other, by default BB
    TArray<FBoundingBox> BBs = GetBBsOfActor(Actor);
    FBoundingBox BB = CombineBBs(BBs);
    // Conver to local space; GetBBsOfActor return BBs in world space
    FTransform Transform = Actor->GetTransform();
    BB.Origin = Transform.InverseTransformPosition(BB.Origin);
    BB.Rotation = Transform.InverseTransformRotation(BB.Rotation.Quaternion()).Rotator();
    return BB;

  }
  return {};
}

FBoundingBox UBoundingBoxCalculator::GetVehicleBoundingBox(
    const ACarlaWheeledVehicle* Vehicle,
    uint8 InTagQueried)
{
  check(Vehicle);

  crp::CityObjectLabel TagQueried = (crp::CityObjectLabel)InTagQueried;
  bool FilterByTagEnabled = (TagQueried != crp::CityObjectLabel::Any);

  UActorComponent *ActorComp = Vehicle->GetComponentByClass(USkeletalMeshComponent::StaticClass());
  USkeletalMeshComponent* ParentComp = Cast<USkeletalMeshComponent>(ActorComp);

  // Filter by tag
  crp::CityObjectLabel Tag = ATagger::GetTagOfTaggedComponent(*ParentComp);
  if(FilterByTagEnabled && Tag != TagQueried) return {};

  USkeletalMesh* SkeletalMesh = ParentComp->SkeletalMesh;
  FBoundingBox BoundingBox = GetSkeletalMeshBoundingBox(SkeletalMesh);

  if(BoundingBox.Extent.IsZero())
  {
    UE_LOG(LogCarla, Error, TEXT("%s has no SKM assigned"), *Vehicle->GetName());
    return {};
  }

  // Two wheel vehicle have more than one SKM
  TArray<USkeletalMeshComponent*> SkeletalMeshComps;
  Vehicle->GetComponents<USkeletalMeshComponent>(SkeletalMeshComps);

  if(SkeletalMeshComps.Num() > 1)
  {
    FVector VehicleLocation = Vehicle->GetActorLocation();
    FRotator VehicleRotation = Vehicle->GetActorRotation();
    float MaxHeadLocation = TNumericLimits<float>::Lowest();
    float MaxKneeLocation = TNumericLimits<float>::Lowest();
    bool SocketFound = false;
    for(USkeletalMeshComponent* Comp : SkeletalMeshComps)
    {
      if(Comp == ParentComp) continue; // ignore vehicle skeleton

      // Location of sockets are in world space
      FVector HeadLocation = Comp->GetSocketLocation("crl_Head__C") - VehicleLocation;
      FVector KneeLocation = Comp->GetSocketLocation("crl_leg__L") - VehicleLocation;

      HeadLocation = VehicleRotation.UnrotateVector(HeadLocation);
      KneeLocation = VehicleRotation.UnrotateVector(KneeLocation);

      MaxHeadLocation = (HeadLocation.Z > MaxHeadLocation) ? HeadLocation.Z : MaxHeadLocation;
      MaxKneeLocation = (KneeLocation.Y > MaxKneeLocation) ? KneeLocation.Y : MaxKneeLocation;

      SocketFound = true;
    }

    if(SocketFound)
    {
      FVector MinVertex = BoundingBox.Origin - BoundingBox.Extent;
      FVector MaxVertex = BoundingBox.Origin + BoundingBox.Extent;
      MaxHeadLocation += 20.0f; // hack, the bone of the head is at he bottom

      MaxVertex.Y = (MaxKneeLocation > MaxVertex.Y) ? MaxKneeLocation : MaxVertex.Y;
      MinVertex.Y = (-MaxKneeLocation < MinVertex.Y) ? MaxKneeLocation : MinVertex.Y;
      MaxVertex.Z = (MaxHeadLocation > MaxVertex.Z) ? MaxHeadLocation : MaxVertex.Z;

      FVector Extent (
        (MaxVertex.X - MinVertex.X) * 0.5f,
        (MaxVertex.Y - MinVertex.Y) * 0.5f,
        (MaxVertex.Z - MinVertex.Z) * 0.5f
      );

      // Calculate middle point
      FVector Origin (
        (MinVertex.X + Extent.X),
        (MinVertex.Y + Extent.Y),
        (MinVertex.Z + Extent.Z)
      );

      BoundingBox.Origin = Origin;
      BoundingBox.Extent = Extent;
    }
  }

  // Component-to-world transform for this component
  const FTransform& CompToWorldTransform = ParentComp->GetComponentTransform();
  BoundingBox = ApplyTransformToBB(BoundingBox, CompToWorldTransform);

  return BoundingBox;
}

FBoundingBox UBoundingBoxCalculator::GetCharacterBoundingBox(
    const ACharacter* Character,
    uint8 InTagQueried)
{
  check(Character);

  crp::CityObjectLabel TagQueried = (crp::CityObjectLabel)InTagQueried;
  bool FilterByTag = TagQueried == crp::CityObjectLabel::Any ||
                     TagQueried == crp::CityObjectLabel::Pedestrians;

  UCapsuleComponent* Capsule = Character->GetCapsuleComponent();


  if (Capsule && FilterByTag)
  {
    const float Radius = Capsule->GetScaledCapsuleRadius();
    const float HalfHeight = Capsule->GetScaledCapsuleHalfHeight();
    FBoundingBox BoundingBox;
    // Characters have the pivot point centered.
    BoundingBox.Origin = {0.0f, 0.0f, 0.0f};
    BoundingBox.Extent = {Radius, Radius, HalfHeight};
    // Component-to-world transform for this component
    const FTransform& CompToWorldTransform = Capsule->GetComponentTransform();
    BoundingBox = ApplyTransformToBB(BoundingBox, CompToWorldTransform);

    return BoundingBox;
  }

  return {};
}

void UBoundingBoxCalculator::GetTrafficLightBoundingBox(
    const ATrafficLightBase* TrafficLight,
    TArray<FBoundingBox>& OutBB,
    TArray<uint8>& OutTag,
    uint8 InTagQueried)
{
  check(TrafficLight);

  TArray<FBoundingBox> BBsOfTL;
  TArray<uint8> TagsOfTL;

  // This kind of a magic number relying that the lights of a TL are not bigger than 100.
  // and we are gonna compare against a squared distance
  const float DistanceThreshold = 100.0f * 100.0f;

  // The BBs of the TL are calculated per light, so we need to merge the full-box
  uint8 TLTag = static_cast<uint8>(crp::CityObjectLabel::TrafficLight);

  CombineBBsOfActor(TrafficLight, BBsOfTL, TagsOfTL, DistanceThreshold, TLTag);
  check(BBsOfTL.Num() == TagsOfTL.Num());

  bool FilterByTagEnabled = (InTagQueried != static_cast<uint8>(crp::CityObjectLabel::Any));

  for(int i = 0; i < BBsOfTL.Num(); i++)
  {
    FBoundingBox& BB = BBsOfTL[i];
    uint8 Tag = TagsOfTL[i];

    if(FilterByTagEnabled && Tag != InTagQueried) continue;

    OutBB.Emplace(BB);
    OutTag.Emplace(Tag);
  }
}

// TODO: update to calculate current animation pose
FBoundingBox UBoundingBoxCalculator::GetSkeletalMeshBoundingBox(const USkeletalMesh* SkeletalMesh)
{
  if(!SkeletalMesh)
  {
    UE_LOG(LogCarla, Error, TEXT("GetSkeletalMeshBoundingBox no SkeletalMesh"));
    return {};
  }

  // Get Vertex postion information from LOD 0 of the Skeletal Mesh
  FSkeletalMeshRenderData* SkeletalMeshRenderData = SkeletalMesh->GetResourceForRendering();
  FSkeletalMeshLODRenderData& LODRenderData = SkeletalMeshRenderData->LODRenderData[0];
  FStaticMeshVertexBuffers& StaticMeshVertexBuffers = LODRenderData.StaticVertexBuffers;
  FPositionVertexBuffer& FPositionVertexBuffer = StaticMeshVertexBuffers.PositionVertexBuffer;
  uint32 NumVertices = FPositionVertexBuffer.GetNumVertices();

  // Look for Skeletal Mesh bounds (vertex perfect)
  FVector MaxVertex(TNumericLimits<float>::Lowest());
  FVector MinVertex(TNumericLimits<float>::Max());
  for(uint32 i = 0; i < NumVertices; i++)
  {
    FVector& Pos = FPositionVertexBuffer.VertexPosition(i);
    MaxVertex.X = (Pos.X > MaxVertex.X) ? Pos.X : MaxVertex.X;
    MaxVertex.Y = (Pos.Y > MaxVertex.Y) ? Pos.Y : MaxVertex.Y;
    MaxVertex.Z = (Pos.Z > MaxVertex.Z) ? Pos.Z : MaxVertex.Z;
    MinVertex.X = (Pos.X < MinVertex.X) ? Pos.X : MinVertex.X;
    MinVertex.Y = (Pos.Y < MinVertex.Y) ? Pos.Y : MinVertex.Y;
    MinVertex.Z = (Pos.Z < MinVertex.Z) ? Pos.Z : MinVertex.Z;
  }

  // Calculate box extent
  FVector Extent (
    (MaxVertex.X - MinVertex.X) * 0.5f,
    (MaxVertex.Y - MinVertex.Y) * 0.5f,
    (MaxVertex.Z - MinVertex.Z) * 0.5f
  );

  // Calculate middle point
  FVector Origin (
    (MinVertex.X + Extent.X),
    (MinVertex.Y + Extent.Y),
    (MinVertex.Z + Extent.Z)
  );

  return {Origin, Extent};
}

FBoundingBox UBoundingBoxCalculator::GetStaticMeshBoundingBox(const UStaticMesh* StaticMesh)
{
  if(!StaticMesh)
  {
    // UE_LOG(LogCarla, Error, TEXT("GetStaticMeshBoundingBox no StaticMesh"));
    return {};
  }

  FBox Box = StaticMesh->GetBoundingBox();
  return {Box.GetCenter(), Box.GetExtent()};

}

void UBoundingBoxCalculator::GetISMBoundingBox(
    UInstancedStaticMeshComponent* ISMComp,
    TArray<FBoundingBox>& OutBoundingBox)
{
  if(!ISMComp)
  {
    UE_LOG(LogCarla, Error, TEXT("GetISMBoundingBox no ISMComp"));
    return;
  }

  const UStaticMesh *Mesh = ISMComp->GetStaticMesh();
  const FBoundingBox SMBoundingBox = GetStaticMeshBoundingBox(Mesh);

  if(!Mesh)
  {
    UE_LOG(LogCarla, Error, TEXT("%s has no SM assigned to the ISM"), *ISMComp->GetOwner()->GetName());
    return;
  }

  const TArray<FInstancedStaticMeshInstanceData>& PerInstanceSMData = ISMComp->PerInstanceSMData;

  const FTransform ParentTransform = ISMComp->GetComponentTransform();

  for(auto& InstSMIData : PerInstanceSMData)
  {
    const FTransform Transform = FTransform(InstSMIData.Transform) * ParentTransform;
    FBoundingBox BoundingBox = ApplyTransformToBB(SMBoundingBox, Transform);

    OutBoundingBox.Add(BoundingBox);
  }

}

void UBoundingBoxCalculator::GetBBsOfStaticMeshComponents(
    const TArray<UStaticMeshComponent*>& StaticMeshComps,
    TArray<FBoundingBox>& OutBB,
    TArray<uint8>& OutTag,
    uint8 InTagQueried)
{
  crp::CityObjectLabel TagQueried = (crp::CityObjectLabel)InTagQueried;
  bool FilterByTagEnabled = (TagQueried != crp::CityObjectLabel::Any);

  for(UStaticMeshComponent* Comp : StaticMeshComps)
  {

    bool isCrosswalk = Comp->GetOwner()->GetName().Contains("crosswalk");

    // Avoid duplication with SMComp and not visible meshes
    if( (!Comp->IsVisible() && !isCrosswalk) || Cast<UInstancedStaticMeshComponent>(Comp))
    {
      continue;
    }

    // Filter by tag
    crp::CityObjectLabel Tag = ATagger::GetTagOfTaggedComponent(*Comp);
    if(FilterByTagEnabled && Tag != TagQueried) continue;

    UStaticMesh* StaticMesh = Comp->GetStaticMesh();
    FBoundingBox BoundingBox = GetStaticMeshBoundingBox(StaticMesh);

    if(StaticMesh)
    {
      // Component-to-world transform for this component
      const FTransform& CompToWorldTransform = Comp->GetComponentTransform();
      BoundingBox = ApplyTransformToBB(BoundingBox, CompToWorldTransform);
      OutBB.Emplace(BoundingBox);
      OutTag.Emplace(static_cast<uint8>(Tag));
    }
  }
}

void UBoundingBoxCalculator::GetBBsOfSkeletalMeshComponents(
    const TArray<USkeletalMeshComponent*>& SkeletalMeshComps,
    TArray<FBoundingBox>& OutBB,
    TArray<uint8>& OutTag,
    uint8 InTagQueried)
{
  crp::CityObjectLabel TagQueried = (crp::CityObjectLabel)InTagQueried;
  bool FilterByTagEnabled = (TagQueried != crp::CityObjectLabel::Any);

  for(USkeletalMeshComponent* Comp : SkeletalMeshComps)
  {
    // Filter by tag
    crp::CityObjectLabel Tag = ATagger::GetTagOfTaggedComponent(*Comp);

    if(!Comp->IsVisible() || (FilterByTagEnabled && Tag != TagQueried)) continue;

    USkeletalMesh* SkeletalMesh = Comp->SkeletalMesh;
    FBoundingBox BoundingBox = GetSkeletalMeshBoundingBox(SkeletalMesh);
    if(BoundingBox.Extent.IsZero())
    {
      UE_LOG(LogCarla, Error, TEXT("%s has no SKM assigned"), *Comp->GetOwner()->GetName());
    }
    else
    {
      // Component-to-world transform for this component
      const FTransform& CompToWorldTransform = Comp->GetComponentTransform();
      BoundingBox = ApplyTransformToBB(BoundingBox, CompToWorldTransform);
      OutBB.Emplace(BoundingBox);
      OutTag.Emplace(static_cast<uint8>(Tag));
    }
  }
}

TArray<FBoundingBox> UBoundingBoxCalculator::GetBoundingBoxOfActors(
  const TArray<AActor*>& Actors,
  uint8 InTagQueried)
{
  TArray<FBoundingBox> Result;
  for(AActor* Actor : Actors)
  {
    TArray<FBoundingBox> BBs = GetBBsOfActor(Actor, InTagQueried);
    Result.Append(BBs.GetData(), BBs.Num());
  }

  return Result;
}

TArray<FBoundingBox> UBoundingBoxCalculator::GetBBsOfActor(
  const AActor* Actor,
  uint8 InTagQueried)
{
  TArray<FBoundingBox> Result;
  TArray<uint8> Tags;
  crp::CityObjectLabel TagQueried = (crp::CityObjectLabel)InTagQueried;
  bool FilterByTagEnabled = (TagQueried != crp::CityObjectLabel::Any);

  FString ClassName = Actor->GetClass()->GetName();

  // Avoid the BP_Procedural_Building to avoid duplication with their child actors
  // When improved the BP_Procedural_Building this maybe should be removed
  // Note: We don't use casting here because the base class is a BP and is easier to do it this way,
  //       than getting the UClass of the BP to cast the actor.
  if( ClassName.Contains("Procedural_Bulding") ) return Result;

  // The vehicle's BP has a low-polystatic mesh for collisions, we should avoid it
  const ACarlaWheeledVehicle* Vehicle = Cast<ACarlaWheeledVehicle>(Actor);
  if (Vehicle)
  {
    FBoundingBox BoundingBox = GetVehicleBoundingBox(Vehicle, InTagQueried);
    if(!BoundingBox.Extent.IsZero())
    {
      Result.Add(BoundingBox);
    }
    return Result;;
  }

  // Pedestrians, we just use the capsule component at the moment.
  const ACharacter* Character = Cast<ACharacter>(Actor);
  if (Character)
  {
    FBoundingBox BoundingBox = GetCharacterBoundingBox(Character, InTagQueried);
    if(!BoundingBox.Extent.IsZero())
    {
      Result.Add(BoundingBox);
    }
    return Result;
  }

  // TrafficLight, we need to join all the BB of the lights in one
  const ATrafficLightBase* TrafficLight = Cast<ATrafficLightBase>(Actor);
  if(TrafficLight)
  {
    GetTrafficLightBoundingBox(TrafficLight, Result, Tags, InTagQueried);
    return Result;
  }

  // Calculate FBoundingBox of ISM
  TArray<UInstancedStaticMeshComponent *> ISMComps;
  Actor->GetComponents<UInstancedStaticMeshComponent>(ISMComps);
  for(UInstancedStaticMeshComponent* Comp: ISMComps)
  {
    // Filter by tag
    crp::CityObjectLabel Tag = ATagger::GetTagOfTaggedComponent(*Comp);
    if(FilterByTagEnabled && Tag != TagQueried) continue;

    GetISMBoundingBox(Comp, Result);
  }

  // Calculate FBoundingBox of SM
  TArray<UStaticMeshComponent*> StaticMeshComps;
  Tags.Reset();
  Actor->GetComponents<UStaticMeshComponent>(StaticMeshComps);
  GetBBsOfStaticMeshComponents(StaticMeshComps, Result, Tags, InTagQueried);

  // Calculate FBoundingBox of SK_M
  TArray<USkeletalMeshComponent*> SkeletalMeshComps;
  Tags.Reset();
  Actor->GetComponents<USkeletalMeshComponent>(SkeletalMeshComps);
  GetBBsOfSkeletalMeshComponents(SkeletalMeshComps, Result, Tags, InTagQueried);

  return Result;
}

void UBoundingBoxCalculator::CombineBBsOfActor(
    const AActor* Actor,
    TArray<FBoundingBox>& OutBB,
    TArray<uint8>& OutTag,
    const float DistanceThreshold,
    uint8 TagToCombine)
{
  check(Actor);

  TArray<FBoundingBox> BBsOfTL;
  TArray<uint8> InternalOutTags;
  TArray<UStaticMeshComponent*> StaticMeshComps;
  Actor->GetComponents<UStaticMeshComponent>(StaticMeshComps);
  GetBBsOfStaticMeshComponents(StaticMeshComps, BBsOfTL, InternalOutTags);

  bool IgnoreDistanceFilter = (DistanceThreshold <= 0.0f);
  bool IgnoreTag = (TagToCombine == 0);

  TSet<int> IndicesDiscarded;
  for(int i = 0; i < BBsOfTL.Num(); i++)
  {
    // Check if the index was used to merge a previous BB
    if(IndicesDiscarded.Contains(i)) continue;

    TArray<FBoundingBox> BBsToCombine;
    FBoundingBox& BB1 = BBsOfTL[i];
    uint8 Tag1 = InternalOutTags[i];

    for(int j = i + 1; j < BBsOfTL.Num(); j++)
    {
      // Check if the index was used to merge a previous BB
      if(IndicesDiscarded.Contains(j)) continue;

      FBoundingBox& BB2 = BBsOfTL[j];
      uint8 Tag2 = InternalOutTags[j];

      float Distance = FVector::DistSquared(BB1.Origin, BB2.Origin);

      // If the BBs are close enough and are from the same type, we merge it
      if( ((Distance <= DistanceThreshold) || IgnoreDistanceFilter) &&
          ((Tag1 == Tag2) || IgnoreTag) )
      {
        BBsToCombine.Emplace(BB2);
        IndicesDiscarded.Emplace(j);
      }
    }
    if(BBsToCombine.Num() > 0)
    {
      BBsToCombine.Emplace(BB1);
      IndicesDiscarded.Emplace(i);
      FBoundingBox MergedBB = CombineBBs(BBsToCombine);
      MergedBB.Rotation = BB1.Rotation;
      OutBB.Emplace(MergedBB);
      OutTag.Emplace(InternalOutTags[i]);
    }
  }

  // Add the BB of the meshes that didn't need to combine
  for(int i = 0; i < BBsOfTL.Num(); i++)
  {
    // Check if the index was used to merge a previous BB
    if(IndicesDiscarded.Contains(i)) continue;
    FBoundingBox& BB = BBsOfTL[i];
    OutBB.Emplace(BB);
    OutTag.Emplace(InternalOutTags[i]);
  }
}

FBoundingBox UBoundingBoxCalculator::CombineBBs(const TArray<FBoundingBox>& BBsToCombine)
{
  FVector MaxVertex(TNumericLimits<float>::Lowest());
  FVector MinVertex(TNumericLimits<float>::Max());

  for(const FBoundingBox& BB : BBsToCombine) {
    FVector MaxVertexOfBB = BB.Origin + BB.Extent;
    FVector MinVertexOfBB = BB.Origin - BB.Extent;

    MaxVertex.X = (MaxVertexOfBB.X > MaxVertex.X) ? MaxVertexOfBB.X : MaxVertex.X;
    MaxVertex.Y = (MaxVertexOfBB.Y > MaxVertex.Y) ? MaxVertexOfBB.Y : MaxVertex.Y;
    MaxVertex.Z = (MaxVertexOfBB.Z > MaxVertex.Z) ? MaxVertexOfBB.Z : MaxVertex.Z;
    MinVertex.X = (MinVertexOfBB.X < MinVertex.X) ? MinVertexOfBB.X : MinVertex.X;
    MinVertex.Y = (MinVertexOfBB.Y < MinVertex.Y) ? MinVertexOfBB.Y : MinVertex.Y;
    MinVertex.Z = (MinVertexOfBB.Z < MinVertex.Z) ? MinVertexOfBB.Z : MinVertex.Z;
  }

  // Calculate box extent
  FVector Extent (
    (MaxVertex.X - MinVertex.X) * 0.5f,
    (MaxVertex.Y - MinVertex.Y) * 0.5f,
    (MaxVertex.Z - MinVertex.Z) * 0.5f
  );

  // Calculate middle point
  FVector Origin (
    (MinVertex.X + Extent.X),
    (MinVertex.Y + Extent.Y),
    (MinVertex.Z + Extent.Z)
  );

  return {Origin, Extent};
}

void UBoundingBoxCalculator::GetMeshCompsFromActorBoundingBox(
  const AActor* Actor,
  const FBoundingBox& InBB,
  TArray<UStaticMeshComponent*>& OutStaticMeshComps)
{
  check(Actor);


  const float DistanceThreshold = 100.0f * 100.0f;

  TArray<UStaticMeshComponent*> SMComps;
  Actor->GetComponents<UStaticMeshComponent>(SMComps);

  for(UStaticMeshComponent* Comp : SMComps)
  {
    if(!Comp->IsVisible()) continue;

    TArray<FBoundingBox> BBs;
    TArray<uint8> InternalOutTags;
    GetBBsOfStaticMeshComponents({Comp}, BBs, InternalOutTags);

    if(BBs.Num() == 0) continue;

    const FBoundingBox& BB = BBs[0];
    float Distance = FVector::DistSquared(InBB.Origin, BB.Origin);
    if(Distance <= DistanceThreshold)
    {
      OutStaticMeshComps.Emplace(Comp);
    }
  }
}
