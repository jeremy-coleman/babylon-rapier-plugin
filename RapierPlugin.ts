import type {
  AbstractMesh,
  IMotorEnabledJoint,
  IPhysicsEnginePlugin,
  PhysicsImpostor,
  PhysicsImpostorJoint,
  PhysicsJoint,
  Quaternion,
  Vector3
} from "babylonjs"
import { PhysicsRaycastResult } from "babylonjs/Physics/physicsRaycastResult"


/**
//https://doc.babylonjs.com/divingDeeper/physics/usingPhysicsEngine
//https://github.com/BabylonJS/Documentation/blob/master/content/How_To/Physics/Using_The_Physics_Engine.md
//https://github.com/BabylonJSGuide/Physics
Each physics engine has different types of Impostors. The following table shows what each engine supports, and what it uses to simulate the missing impostors
| Impostor Type | Cannon.js | Oimo.js | Energy.js | Ammo.js | Notes   |
|---------------|-----------|---------|-----------|---------|---------|
| Box           | Box       | Box     | Box       | Box     |         |
| Sphere        | Sphere    | Sphere  | Sphere    | Sphere  |         |
| Particle      | Particle  | Sphere  | Unknown   | Sphere  |         |
| Plane         | Plane     | Box     | Plane     | Box     | Simulates an unlimited surface. Like a floor that never ends. Consider using Box |
| Cylinder      | Cylinder  | Cylinder| Cylinder  | Cylinder|         |
| Mesh          | Mesh      | Box     | Mesh      | Mesh    | Use only when necessary - will lower performance. Cannon's mesh impostor only collides against spheres and planes |
| Heightmap     | Heightmap | Box     | Mesh      | Mesh    |         |
| ConvexHull    | N/A       | N/A     | N/A       | Mesh    | Allows physics impostor support for convex mesh hull shapes |
*/

//enum used in the registerMesh method
var IMPOSTER = {
  NoImpostor: 0,
  SphereImpostor: 1,
  BoxImpostor: 2,
  PlaneImpostor: 3,
  CompoundImpostor: 4,
  MeshImpostor: 4
}

class RapierPlugin implements IPhysicsEnginePlugin {
  world: any
  name: string
  setGravity(gravity: Vector3): void {
    throw new Error("Method not implemented.")
  }
  setTimeStep(timeStep: number): void {
    throw new Error("Method not implemented.")
  }
  getTimeStep(): number {
    throw new Error("Method not implemented.")
  }
  executeStep(delta: number, impostors: PhysicsImpostor[]): void {
    throw new Error("Method not implemented.")
  }
  applyImpulse(impostor: PhysicsImpostor, force: Vector3, contactPoint: Vector3): void {
    throw new Error("Method not implemented.")
  }
  applyForce(impostor: PhysicsImpostor, force: Vector3, contactPoint: Vector3): void {
    throw new Error("Method not implemented.")
  }
  generatePhysicsBody(impostor: PhysicsImpostor): void {
    throw new Error("Method not implemented.")
  }
  removePhysicsBody(impostor: PhysicsImpostor): void {
    throw new Error("Method not implemented.")
  }
  generateJoint(joint: PhysicsImpostorJoint): void {
    throw new Error("Method not implemented.")
  }
  removeJoint(joint: PhysicsImpostorJoint): void {
    throw new Error("Method not implemented.")
  }
  isSupported(): boolean {
    throw new Error("Method not implemented.")
  }
  setTransformationFromPhysicsBody(impostor: PhysicsImpostor): void {
    throw new Error("Method not implemented.")
  }
  setPhysicsBodyTransformation(impostor: PhysicsImpostor, newPosition: Vector3, newRotation: Quaternion): void {
    throw new Error("Method not implemented.")
  }
  setLinearVelocity(impostor: PhysicsImpostor, velocity: Vector3): void {
    throw new Error("Method not implemented.")
  }
  setAngularVelocity(impostor: PhysicsImpostor, velocity: Vector3): void {
    throw new Error("Method not implemented.")
  }
  getLinearVelocity(impostor: PhysicsImpostor): Vector3 {
    throw new Error("Method not implemented.")
  }
  getAngularVelocity(impostor: PhysicsImpostor): Vector3 {
    throw new Error("Method not implemented.")
  }
  setBodyMass(impostor: PhysicsImpostor, mass: number): void {
    throw new Error("Method not implemented.")
  }
  getBodyMass(impostor: PhysicsImpostor): number {
    throw new Error("Method not implemented.")
  }
  getBodyFriction(impostor: PhysicsImpostor): number {
    throw new Error("Method not implemented.")
  }
  setBodyFriction(impostor: PhysicsImpostor, friction: number): void {
    throw new Error("Method not implemented.")
  }
  getBodyRestitution(impostor: PhysicsImpostor): number {
    throw new Error("Method not implemented.")
  }
  setBodyRestitution(impostor: PhysicsImpostor, restitution: number): void {
    throw new Error("Method not implemented.")
  }
  getBodyPressure?(impostor: PhysicsImpostor): number {
    throw new Error("Method not implemented.")
  }
  setBodyPressure?(impostor: PhysicsImpostor, pressure: number): void {
    throw new Error("Method not implemented.")
  }
  getBodyStiffness?(impostor: PhysicsImpostor): number {
    throw new Error("Method not implemented.")
  }
  setBodyStiffness?(impostor: PhysicsImpostor, stiffness: number): void {
    throw new Error("Method not implemented.")
  }
  getBodyVelocityIterations?(impostor: PhysicsImpostor): number {
    throw new Error("Method not implemented.")
  }
  setBodyVelocityIterations?(impostor: PhysicsImpostor, velocityIterations: number): void {
    throw new Error("Method not implemented.")
  }
  getBodyPositionIterations?(impostor: PhysicsImpostor): number {
    throw new Error("Method not implemented.")
  }
  setBodyPositionIterations?(impostor: PhysicsImpostor, positionIterations: number): void {
    throw new Error("Method not implemented.")
  }
  appendAnchor?(
    impostor: PhysicsImpostor,
    otherImpostor: PhysicsImpostor,
    width: number,
    height: number,
    influence: number,
    noCollisionBetweenLinkedBodies: boolean
  ): void {
    throw new Error("Method not implemented.")
  }
  appendHook?(
    impostor: PhysicsImpostor,
    otherImpostor: PhysicsImpostor,
    length: number,
    influence: number,
    noCollisionBetweenLinkedBodies: boolean
  ): void {
    throw new Error("Method not implemented.")
  }
  sleepBody(impostor: PhysicsImpostor): void {
    throw new Error("Method not implemented.")
  }
  wakeUpBody(impostor: PhysicsImpostor): void {
    throw new Error("Method not implemented.")
  }
  raycast(from: Vector3, to: Vector3): PhysicsRaycastResult {
    throw new Error("Method not implemented.")
  }
  updateDistanceJoint(joint: PhysicsJoint, maxDistance: number, minDistance?: number): void {
    throw new Error("Method not implemented.")
  }
  setMotor(joint: IMotorEnabledJoint, speed: number, maxForce?: number, motorIndex?: number): void {
    throw new Error("Method not implemented.")
  }
  setLimit(joint: IMotorEnabledJoint, upperLimit: number, lowerLimit?: number, motorIndex?: number): void {
    throw new Error("Method not implemented.")
  }
  getRadius(impostor: PhysicsImpostor): number {
    throw new Error("Method not implemented.")
  }
  getBoxSizeToRef(impostor: PhysicsImpostor, result: Vector3): void {
    throw new Error("Method not implemented.")
  }
  syncMeshWithImpostor(mesh: AbstractMesh, impostor: PhysicsImpostor): void {
    throw new Error("Method not implemented.")
  }
  dispose(): void {
    throw new Error("Method not implemented.")
  }

  initialize() {} //: Must initialize your engine

  //babylon.js will call this function for each frame, giving you the delta time between current and previous frame.This is the responsibility of the plugin to update meshes' position and rotation accordingly to the physics simulation.
  runOneStep(delta) {}

  //Called to remove a mesh from the simulation
  unregisterMesh(mesh) {}

  //babylon.js will call this function when the user wants to create a physics impostor for a mesh.options parameter contains 3 values: mass, friction and restitution.Possible values for impostor are the following:
  registerMesh(mesh, impostor, options) {}

  //Babylon.js will call this function for compound objects.parts parameter contains an array of { mesh, impostor }. options parameter is the same as above.
  registerMeshesAsCompound(parts, options) {}

  //: Create a joint between two meshes
  createLink(mesh1, mesh2, pivot1, pivot2) {}
}

//these 4 methods were in the 10 mandatory methods stated in the docs but not present in the IPhysicsEnginePlugin interface
//applyImpulse(mesh, force, contactPoint){}
//setGravity(gravity){}
//dispose(){}//isSupported(){}

//these are all the methods mentioned in the docs
// initialize()
// setGravity(gravity)
// runOneStep(delta)
// registerMesh(mesh, impostor, options)
// registerMeshesAsCompound(parts, options)
// unregisterMesh(mesh)
// applyImpulse(mesh, force, contactPoint)
// createLink(mesh1, mesh2, pivot1, pivot2)
// dispose()
// isSupported()
