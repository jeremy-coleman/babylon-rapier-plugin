//copy/pasted from npm version 0.0.8-alpha.0
/* tslint:disable */
/* eslint-disable */
/**
* @returns {string}
*/
export function version(): string;
/**
*/
export enum RawJointType {
  Spherical,
  Fixed,
  Prismatic,
  Revolute,
  Generic,
}
/**
*/
export enum RawMotorModel {
  VelocityBased,
  AccelerationBased,
}
/**
*/
export enum RawJointAxis {
  X,
  Y,
  Z,
  AngX,
  AngY,
  AngZ,
}
/**
*/
export enum RawRigidBodyType {
  Dynamic,
  Static,
  KinematicPositionBased,
  KinematicVelocityBased,
}
/**
*/
export enum RawShapeType {
  Ball,
  Cuboid,
  Capsule,
  Segment,
  Polyline,
  Triangle,
  TriMesh,
  HeightField,
  Compound,
  ConvexPolyhedron,
  Cylinder,
  Cone,
  RoundCuboid,
  RoundTriangle,
  RoundCylinder,
  RoundCone,
  RoundConvexPolyhedron,
}
/**
*/
export class RawBroadPhase {
  free(): void;
/**
*/
  constructor();
}
/**
*/
export class RawCCDSolver {
  free(): void;
/**
*/
  constructor();
}
/**
*/
export class RawColliderSet {
  free(): void;
/**
* The world-space translation of this collider.
* @param {number} handle
* @returns {RawVector}
*/
  coTranslation(handle: number): RawVector;
/**
* The world-space orientation of this collider.
* @param {number} handle
* @returns {RawRotation}
*/
  coRotation(handle: number): RawRotation;
/**
* Sets the translation of this collider.
*
* # Parameters
* - `x`: the world-space position of the collider along the `x` axis.
* - `y`: the world-space position of the collider along the `y` axis.
* - `z`: the world-space position of the collider along the `z` axis.
* - `wakeUp`: forces the collider to wake-up so it is properly affected by forces if it
* wasn't moving before modifying its position.
* @param {number} handle
* @param {number} x
* @param {number} y
* @param {number} z
*/
  coSetTranslation(handle: number, x: number, y: number, z: number): void;
/**
* @param {number} handle
* @param {number} x
* @param {number} y
* @param {number} z
*/
  coSetTranslationWrtParent(handle: number, x: number, y: number, z: number): void;
/**
* Sets the rotation quaternion of this collider.
*
* This does nothing if a zero quaternion is provided.
*
* # Parameters
* - `x`: the first vector component of the quaternion.
* - `y`: the second vector component of the quaternion.
* - `z`: the third vector component of the quaternion.
* - `w`: the scalar component of the quaternion.
* - `wakeUp`: forces the collider to wake-up so it is properly affected by forces if it
* wasn't moving before modifying its position.
* @param {number} handle
* @param {number} x
* @param {number} y
* @param {number} z
* @param {number} w
*/
  coSetRotation(handle: number, x: number, y: number, z: number, w: number): void;
/**
* @param {number} handle
* @param {number} x
* @param {number} y
* @param {number} z
* @param {number} w
*/
  coSetRotationWrtParent(handle: number, x: number, y: number, z: number, w: number): void;
/**
* Is this collider a sensor?
* @param {number} handle
* @returns {boolean}
*/
  coIsSensor(handle: number): boolean;
/**
* The type of the shape of this collider.
* @param {number} handle
* @returns {number}
*/
  coShapeType(handle: number): number;
/**
* The half-extents of this collider if it is has a cuboid shape.
* @param {number} handle
* @returns {RawVector | undefined}
*/
  coHalfExtents(handle: number): RawVector | undefined;
/**
* The radius of this collider if it is a ball, capsule, cylinder, or cone shape.
* @param {number} handle
* @returns {number | undefined}
*/
  coRadius(handle: number): number | undefined;
/**
* The radius of this collider if it is a capsule, cylinder, or cone shape.
* @param {number} handle
* @returns {number | undefined}
*/
  coHalfHeight(handle: number): number | undefined;
/**
* The radius of the round edges of this collider if it is a round cylinder.
* @param {number} handle
* @returns {number | undefined}
*/
  coRoundRadius(handle: number): number | undefined;
/**
* The vertices of this triangle mesh, polyline, convex polyhedron, or convex polyhedron, if it is one.
* @param {number} handle
* @returns {Float32Array | undefined}
*/
  coVertices(handle: number): Float32Array | undefined;
/**
* The indices of this triangle mesh, polyline, or convex polyhedron, if it is one.
* @param {number} handle
* @returns {Uint32Array | undefined}
*/
  coIndices(handle: number): Uint32Array | undefined;
/**
* The height of this heightfield if it is one.
* @param {number} handle
* @returns {Float32Array | undefined}
*/
  coHeightfieldHeights(handle: number): Float32Array | undefined;
/**
* The scaling factor applied of this heightfield if it is one.
* @param {number} handle
* @returns {RawVector | undefined}
*/
  coHeightfieldScale(handle: number): RawVector | undefined;
/**
* The number of rows on this heightfield's height matrix, if it is one.
* @param {number} handle
* @returns {number | undefined}
*/
  coHeightfieldNRows(handle: number): number | undefined;
/**
* The number of columns on this heightfield's height matrix, if it is one.
* @param {number} handle
* @returns {number | undefined}
*/
  coHeightfieldNCols(handle: number): number | undefined;
/**
* The unique integer identifier of the collider this collider is attached to.
* @param {number} handle
* @returns {number}
*/
  coParent(handle: number): number;
/**
* The friction coefficient of this collider.
* @param {number} handle
* @returns {number}
*/
  coFriction(handle: number): number;
/**
* The restitution coefficient of this collider.
* @param {number} handle
* @returns {number}
*/
  coRestitution(handle: number): number;
/**
* The density of this collider.
* @param {number} handle
* @returns {number | undefined}
*/
  coDensity(handle: number): number | undefined;
/**
* The collision groups of this collider.
* @param {number} handle
* @returns {number}
*/
  coCollisionGroups(handle: number): number;
/**
* The solver groups of this collider.
* @param {number} handle
* @returns {number}
*/
  coSolverGroups(handle: number): number;
/**
* The physics hooks enabled for this collider.
* @param {number} handle
* @returns {number}
*/
  coActiveHooks(handle: number): number;
/**
* The collision types enabled for this collider.
* @param {number} handle
* @returns {number}
*/
  coActiveCollisionTypes(handle: number): number;
/**
* The events enabled for this collider.
* @param {number} handle
* @returns {number}
*/
  coActiveEvents(handle: number): number;
/**
* @param {number} handle
* @param {boolean} is_sensor
*/
  coSetSensor(handle: number, is_sensor: boolean): void;
/**
* @param {number} handle
* @param {number} restitution
*/
  coSetRestitution(handle: number, restitution: number): void;
/**
* @param {number} handle
* @param {number} friction
*/
  coSetFriction(handle: number, friction: number): void;
/**
* @param {number} handle
* @returns {number}
*/
  coFrictionCombineRule(handle: number): number;
/**
* @param {number} handle
* @param {number} rule
*/
  coSetFrictionCombineRule(handle: number, rule: number): void;
/**
* @param {number} handle
* @returns {number}
*/
  coRestitutionCombineRule(handle: number): number;
/**
* @param {number} handle
* @param {number} rule
*/
  coSetRestitutionCombineRule(handle: number, rule: number): void;
/**
* @param {number} handle
* @param {number} groups
*/
  coSetCollisionGroups(handle: number, groups: number): void;
/**
* @param {number} handle
* @param {number} groups
*/
  coSetSolverGroups(handle: number, groups: number): void;
/**
* @param {number} handle
* @param {number} hooks
*/
  coSetActiveHooks(handle: number, hooks: number): void;
/**
* @param {number} handle
* @param {number} events
*/
  coSetActiveEvents(handle: number, events: number): void;
/**
* @param {number} handle
* @param {number} types
*/
  coSetActiveCollisionTypes(handle: number, types: number): void;
/**
* @param {number} handle
* @param {RawShape} shape
*/
  coSetShape(handle: number, shape: RawShape): void;
/**
*/
  constructor();
/**
* @returns {number}
*/
  len(): number;
/**
* @param {number} handle
* @returns {boolean}
*/
  contains(handle: number): boolean;
/**
* @param {RawShape} shape
* @param {RawVector} translation
* @param {RawRotation} rotation
* @param {boolean} useMassProps
* @param {number} mass
* @param {RawVector} centerOfMass
* @param {RawVector} principalAngularInertia
* @param {RawRotation} angularInertiaFrame
* @param {number} density
* @param {number} friction
* @param {number} restitution
* @param {number} frictionCombineRule
* @param {number} restitutionCombineRule
* @param {boolean} isSensor
* @param {number} collisionGroups
* @param {number} solverGroups
* @param {number} activeCollisionTypes
* @param {number} activeHooks
* @param {number} activeEvents
* @param {boolean} hasParent
* @param {number} parent
* @param {RawRigidBodySet} bodies
* @returns {number | undefined}
*/
  createCollider(shape: RawShape, translation: RawVector, rotation: RawRotation, useMassProps: boolean, mass: number, centerOfMass: RawVector, principalAngularInertia: RawVector, angularInertiaFrame: RawRotation, density: number, friction: number, restitution: number, frictionCombineRule: number, restitutionCombineRule: number, isSensor: boolean, collisionGroups: number, solverGroups: number, activeCollisionTypes: number, activeHooks: number, activeEvents: number, hasParent: boolean, parent: number, bodies: RawRigidBodySet): number | undefined;
/**
* Removes a collider from this set and wake-up the rigid-body it is attached to.
* @param {number} handle
* @param {RawIslandManager} islands
* @param {RawRigidBodySet} bodies
* @param {boolean} wakeUp
*/
  remove(handle: number, islands: RawIslandManager, bodies: RawRigidBodySet, wakeUp: boolean): void;
/**
* Checks if a collider with the given integer handle exists.
* @param {number} handle
* @returns {boolean}
*/
  isHandleValid(handle: number): boolean;
/**
* Applies the given JavaScript function to the integer handle of each collider managed by this collider set.
*
* # Parameters
* - `f(handle)`: the function to apply to the integer handle of each collider managed by this collider set. Called as `f(handle)`.
* @param {Function} f
*/
  forEachColliderHandle(f: Function): void;
}
/**
*/
export class RawContactManifold {
  free(): void;
/**
* @returns {RawVector}
*/
  normal(): RawVector;
/**
* @returns {RawVector}
*/
  local_n1(): RawVector;
/**
* @returns {RawVector}
*/
  local_n2(): RawVector;
/**
* @returns {number}
*/
  subshape1(): number;
/**
* @returns {number}
*/
  subshape2(): number;
/**
* @returns {number}
*/
  num_contacts(): number;
/**
* @param {number} i
* @returns {RawVector | undefined}
*/
  contact_local_p1(i: number): RawVector | undefined;
/**
* @param {number} i
* @returns {RawVector | undefined}
*/
  contact_local_p2(i: number): RawVector | undefined;
/**
* @param {number} i
* @returns {number}
*/
  contact_dist(i: number): number;
/**
* @param {number} i
* @returns {number}
*/
  contact_fid1(i: number): number;
/**
* @param {number} i
* @returns {number}
*/
  contact_fid2(i: number): number;
/**
* @param {number} i
* @returns {number}
*/
  contact_impulse(i: number): number;
/**
* @param {number} i
* @returns {number}
*/
  contact_tangent_impulse_x(i: number): number;
/**
* @param {number} i
* @returns {number}
*/
  contact_tangent_impulse_y(i: number): number;
/**
* @returns {number}
*/
  num_solver_contacts(): number;
/**
* @param {number} i
* @returns {RawVector | undefined}
*/
  solver_contact_point(i: number): RawVector | undefined;
/**
* @param {number} i
* @returns {number}
*/
  solver_contact_dist(i: number): number;
/**
* @param {number} i
* @returns {number}
*/
  solver_contact_friction(i: number): number;
/**
* @param {number} i
* @returns {number}
*/
  solver_contact_restitution(i: number): number;
/**
* @param {number} i
* @returns {RawVector}
*/
  solver_contact_tangent_velocity(i: number): RawVector;
}
/**
*/
export class RawContactPair {
  free(): void;
/**
* @returns {number}
*/
  collider1(): number;
/**
* @returns {number}
*/
  collider2(): number;
/**
* @returns {number}
*/
  numContactManifolds(): number;
/**
* @param {number} i
* @returns {RawContactManifold | undefined}
*/
  contactManifold(i: number): RawContactManifold | undefined;
}
/**
*/
export class RawDeserializedWorld {
  free(): void;
/**
* @returns {RawVector | undefined}
*/
  takeGravity(): RawVector | undefined;
/**
* @returns {RawIntegrationParameters | undefined}
*/
  takeIntegrationParameters(): RawIntegrationParameters | undefined;
/**
* @returns {RawIslandManager | undefined}
*/
  takeIslandManager(): RawIslandManager | undefined;
/**
* @returns {RawBroadPhase | undefined}
*/
  takeBroadPhase(): RawBroadPhase | undefined;
/**
* @returns {RawNarrowPhase | undefined}
*/
  takeNarrowPhase(): RawNarrowPhase | undefined;
/**
* @returns {RawRigidBodySet | undefined}
*/
  takeBodies(): RawRigidBodySet | undefined;
/**
* @returns {RawColliderSet | undefined}
*/
  takeColliders(): RawColliderSet | undefined;
/**
* @returns {RawImpulseJointSet | undefined}
*/
  takeImpulseJoints(): RawImpulseJointSet | undefined;
/**
* @returns {RawMultibodyJointSet | undefined}
*/
  takeMultibodyJoints(): RawMultibodyJointSet | undefined;
}
/**
* A structure responsible for collecting events generated
* by the physics engine.
*/
export class RawEventQueue {
  free(): void;
/**
* Creates a new event collector.
*
* # Parameters
* - `autoDrain`: setting this to `true` is strongly recommended. If true, the collector will
* be automatically drained before each `world.step(collector)`. If false, the collector will
* keep all events in memory unless it is manually drained/cleared; this may lead to unbounded use of
* RAM if no drain is performed.
* @param {boolean} autoDrain
*/
  constructor(autoDrain: boolean);
/**
* Applies the given javascript closure on each contact event of this collector, then clear
* the internal contact event buffer.
*
* # Parameters
* - `f(handle1, handle2, started)`:  JavaScript closure applied to each contact event. The
* closure should take three arguments: two integers representing the handles of the colliders
* involved in the contact, and a boolean indicating if the contact started (true) or stopped
* (false).
* @param {Function} f
*/
  drainContactEvents(f: Function): void;
/**
* Applies the given javascript closure on each proximity event of this collector, then clear
* the internal proximity event buffer.
*
* # Parameters
* - `f(handle1, handle2, prev_prox, new_prox)`:  JavaScript closure applied to each proximity event. The
* closure should take four arguments: two integers representing the handles of the colliders
* involved in the proximity, and one boolean representing the intersection status.
* @param {Function} f
*/
  drainIntersectionEvents(f: Function): void;
/**
* Removes all events contained by this collector.
*/
  clear(): void;
}
/**
*/
export class RawImpulseJointSet {
  free(): void;
/**
* The type of this joint.
* @param {number} handle
* @returns {number}
*/
  jointType(handle: number): number;
/**
* The unique integer identifier of the first rigid-body this joint it attached to.
* @param {number} handle
* @returns {number}
*/
  jointBodyHandle1(handle: number): number;
/**
* The unique integer identifier of the second rigid-body this joint is attached to.
* @param {number} handle
* @returns {number}
*/
  jointBodyHandle2(handle: number): number;
/**
* The angular part of the joint’s local frame relative to the first rigid-body it is attached to.
* @param {number} handle
* @returns {RawRotation}
*/
  jointFrameX1(handle: number): RawRotation;
/**
* The angular part of the joint’s local frame relative to the second rigid-body it is attached to.
* @param {number} handle
* @returns {RawRotation}
*/
  jointFrameX2(handle: number): RawRotation;
/**
* The position of the first anchor of this joint.
*
* The first anchor gives the position of the points application point on the
* local frame of the first rigid-body it is attached to.
* @param {number} handle
* @returns {RawVector}
*/
  jointAnchor1(handle: number): RawVector;
/**
* The position of the second anchor of this joint.
*
* The second anchor gives the position of the points application point on the
* local frame of the second rigid-body it is attached to.
* @param {number} handle
* @returns {RawVector}
*/
  jointAnchor2(handle: number): RawVector;
/**
* Are the limits for this joint enabled?
* @param {number} handle
* @param {number} axis
* @returns {boolean}
*/
  jointLimitsEnabled(handle: number, axis: number): boolean;
/**
* Return the lower limit along the given joint axis.
* @param {number} handle
* @param {number} axis
* @returns {number}
*/
  jointLimitsMin(handle: number, axis: number): number;
/**
* If this is a prismatic joint, returns its upper limit.
* @param {number} handle
* @param {number} axis
* @returns {number}
*/
  jointLimitsMax(handle: number, axis: number): number;
/**
* @param {number} handle
* @param {number} axis
* @param {number} model
*/
  jointConfigureMotorModel(handle: number, axis: number, model: number): void;
/**
* @param {number} handle
* @param {number} axis
* @param {number} targetVel
* @param {number} factor
*/
  jointConfigureMotorVelocity(handle: number, axis: number, targetVel: number, factor: number): void;
/**
* @param {number} handle
* @param {number} axis
* @param {number} targetPos
* @param {number} stiffness
* @param {number} damping
*/
  jointConfigureMotorPosition(handle: number, axis: number, targetPos: number, stiffness: number, damping: number): void;
/**
* @param {number} handle
* @param {number} axis
* @param {number} targetPos
* @param {number} targetVel
* @param {number} stiffness
* @param {number} damping
*/
  jointConfigureMotor(handle: number, axis: number, targetPos: number, targetVel: number, stiffness: number, damping: number): void;
/**
*/
  constructor();
/**
* @param {RawRigidBodySet} bodies
* @param {RawJointData} params
* @param {number} parent1
* @param {number} parent2
* @returns {number}
*/
  createJoint(bodies: RawRigidBodySet, params: RawJointData, parent1: number, parent2: number): number;
/**
* @param {number} handle
* @param {RawIslandManager} islands
* @param {RawRigidBodySet} bodies
* @param {boolean} wakeUp
*/
  remove(handle: number, islands: RawIslandManager, bodies: RawRigidBodySet, wakeUp: boolean): void;
/**
* @returns {number}
*/
  len(): number;
/**
* @param {number} handle
* @returns {boolean}
*/
  contains(handle: number): boolean;
/**
* Applies the given JavaScript function to the integer handle of each joint managed by this physics world.
*
* # Parameters
* - `f(handle)`: the function to apply to the integer handle of each joint managed by this set. Called as `f(collider)`.
* @param {Function} f
*/
  forEachJointHandle(f: Function): void;
}
/**
*/
export class RawIntegrationParameters {
  free(): void;
/**
*/
  constructor();
/**
* @returns {number}
*/
  allowedLinearError: number;
/**
* @returns {number}
*/
  dt: number;
/**
* @returns {number}
*/
  erp: number;
/**
* @returns {number}
*/
  maxCcdSubsteps: number;
/**
* @returns {number}
*/
  maxStabilizationIterations: number;
/**
* @returns {number}
*/
  maxVelocityFrictionIterations: number;
/**
* @returns {number}
*/
  maxVelocityIterations: number;
/**
* @returns {number}
*/
  minIslandSize: number;
/**
* @returns {number}
*/
  predictionDistance: number;
}
/**
*/
export class RawIslandManager {
  free(): void;
/**
*/
  constructor();
/**
* Applies the given JavaScript function to the integer handle of each active rigid-body
* managed by this island manager.
*
* After a short time of inactivity, a rigid-body is automatically deactivated ("asleep") by
* the physics engine in order to save computational power. A sleeping rigid-body never moves
* unless it is moved manually by the user.
*
* # Parameters
* - `f(handle)`: the function to apply to the integer handle of each active rigid-body managed by this
*   set. Called as `f(collider)`.
* @param {Function} f
*/
  forEachActiveRigidBodyHandle(f: Function): void;
}
/**
*/
export class RawJointData {
  free(): void;
/**
* Create a new joint descriptor that builds spehrical joints.
*
* A spherical joints allows three relative rotational degrees of freedom
* by preventing any relative translation between the anchors of the
* two attached rigid-bodies.
* @param {RawVector} anchor1
* @param {RawVector} anchor2
* @returns {RawJointData}
*/
  static spherical(anchor1: RawVector, anchor2: RawVector): RawJointData;
/**
* Creates a new joint descriptor that builds a Prismatic joint.
*
* A prismatic joint removes all the degrees of freedom between the
* affected bodies, except for the translation along one axis.
*
* Returns `None` if any of the provided axes cannot be normalized.
* @param {RawVector} anchor1
* @param {RawVector} anchor2
* @param {RawVector} axis
* @param {boolean} limitsEnabled
* @param {number} limitsMin
* @param {number} limitsMax
* @returns {RawJointData | undefined}
*/
  static prismatic(anchor1: RawVector, anchor2: RawVector, axis: RawVector, limitsEnabled: boolean, limitsMin: number, limitsMax: number): RawJointData | undefined;
/**
* Creates a new joint descriptor that builds a Fixed joint.
*
* A fixed joint removes all the degrees of freedom between the affected bodies.
* @param {RawVector} anchor1
* @param {RawRotation} axes1
* @param {RawVector} anchor2
* @param {RawRotation} axes2
* @returns {RawJointData}
*/
  static fixed(anchor1: RawVector, axes1: RawRotation, anchor2: RawVector, axes2: RawRotation): RawJointData;
/**
* Create a new joint descriptor that builds Revolute joints.
*
* A revolute joint removes all degrees of freedom between the affected
* bodies except for the rotation along one axis.
* @param {RawVector} anchor1
* @param {RawVector} anchor2
* @param {RawVector} axis
* @returns {RawJointData | undefined}
*/
  static revolute(anchor1: RawVector, anchor2: RawVector, axis: RawVector): RawJointData | undefined;
}
/**
*/
export class RawMultibodyJointSet {
  free(): void;
/**
* The type of this joint.
* @param {number} handle
* @returns {number}
*/
  jointType(handle: number): number;
/**
* The angular part of the joint’s local frame relative to the first rigid-body it is attached to.
* @param {number} handle
* @returns {RawRotation}
*/
  jointFrameX1(handle: number): RawRotation;
/**
* The angular part of the joint’s local frame relative to the second rigid-body it is attached to.
* @param {number} handle
* @returns {RawRotation}
*/
  jointFrameX2(handle: number): RawRotation;
/**
* The position of the first anchor of this joint.
*
* The first anchor gives the position of the points application point on the
* local frame of the first rigid-body it is attached to.
* @param {number} handle
* @returns {RawVector}
*/
  jointAnchor1(handle: number): RawVector;
/**
* The position of the second anchor of this joint.
*
* The second anchor gives the position of the points application point on the
* local frame of the second rigid-body it is attached to.
* @param {number} handle
* @returns {RawVector}
*/
  jointAnchor2(handle: number): RawVector;
/**
* Are the limits for this joint enabled?
* @param {number} handle
* @param {number} axis
* @returns {boolean}
*/
  jointLimitsEnabled(handle: number, axis: number): boolean;
/**
* Return the lower limit along the given joint axis.
* @param {number} handle
* @param {number} axis
* @returns {number}
*/
  jointLimitsMin(handle: number, axis: number): number;
/**
* If this is a prismatic joint, returns its upper limit.
* @param {number} handle
* @param {number} axis
* @returns {number}
*/
  jointLimitsMax(handle: number, axis: number): number;
/**
*/
  constructor();
/**
* @param {RawRigidBodySet} bodies
* @param {RawJointData} params
* @param {number} parent1
* @param {number} parent2
* @returns {number}
*/
  createJoint(bodies: RawRigidBodySet, params: RawJointData, parent1: number, parent2: number): number;
/**
* @param {number} handle
* @param {RawIslandManager} islands
* @param {RawRigidBodySet} bodies
* @param {boolean} wakeUp
*/
  remove(handle: number, islands: RawIslandManager, bodies: RawRigidBodySet, wakeUp: boolean): void;
/**
* @param {number} handle
* @returns {boolean}
*/
  contains(handle: number): boolean;
/**
* Applies the given JavaScript function to the integer handle of each joint managed by this physics world.
*
* # Parameters
* - `f(handle)`: the function to apply to the integer handle of each joint managed by this set. Called as `f(collider)`.
* @param {Function} f
*/
  forEachJointHandle(f: Function): void;
}
/**
*/
export class RawNarrowPhase {
  free(): void;
/**
*/
  constructor();
/**
* @param {number} handle1
* @param {Function} f
*/
  contacts_with(handle1: number, f: Function): void;
/**
* @param {number} handle1
* @param {number} handle2
* @returns {RawContactPair | undefined}
*/
  contact_pair(handle1: number, handle2: number): RawContactPair | undefined;
/**
* @param {number} handle1
* @param {Function} f
*/
  intersections_with(handle1: number, f: Function): void;
/**
* @param {number} handle1
* @param {number} handle2
* @returns {boolean}
*/
  intersection_pair(handle1: number, handle2: number): boolean;
}
/**
*/
export class RawPhysicsPipeline {
  free(): void;
/**
*/
  constructor();
/**
* @param {RawVector} gravity
* @param {RawIntegrationParameters} integrationParameters
* @param {RawIslandManager} islands
* @param {RawBroadPhase} broadPhase
* @param {RawNarrowPhase} narrowPhase
* @param {RawRigidBodySet} bodies
* @param {RawColliderSet} colliders
* @param {RawImpulseJointSet} joints
* @param {RawMultibodyJointSet} articulations
* @param {RawCCDSolver} ccd_solver
*/
  step(gravity: RawVector, integrationParameters: RawIntegrationParameters, islands: RawIslandManager, broadPhase: RawBroadPhase, narrowPhase: RawNarrowPhase, bodies: RawRigidBodySet, colliders: RawColliderSet, joints: RawImpulseJointSet, articulations: RawMultibodyJointSet, ccd_solver: RawCCDSolver): void;
/**
* @param {RawVector} gravity
* @param {RawIntegrationParameters} integrationParameters
* @param {RawIslandManager} islands
* @param {RawBroadPhase} broadPhase
* @param {RawNarrowPhase} narrowPhase
* @param {RawRigidBodySet} bodies
* @param {RawColliderSet} colliders
* @param {RawImpulseJointSet} joints
* @param {RawMultibodyJointSet} articulations
* @param {RawCCDSolver} ccd_solver
* @param {RawEventQueue} eventQueue
* @param {object} hookObject
* @param {Function} hookFilterContactPair
* @param {Function} hookFilterIntersectionPair
*/
  stepWithEvents(gravity: RawVector, integrationParameters: RawIntegrationParameters, islands: RawIslandManager, broadPhase: RawBroadPhase, narrowPhase: RawNarrowPhase, bodies: RawRigidBodySet, colliders: RawColliderSet, joints: RawImpulseJointSet, articulations: RawMultibodyJointSet, ccd_solver: RawCCDSolver, eventQueue: RawEventQueue, hookObject: object, hookFilterContactPair: Function, hookFilterIntersectionPair: Function): void;
}
/**
*/
export class RawPointColliderProjection {
  free(): void;
/**
* @returns {number}
*/
  colliderHandle(): number;
/**
* @returns {RawVector}
*/
  point(): RawVector;
/**
* @returns {boolean}
*/
  isInside(): boolean;
}
/**
*/
export class RawQueryPipeline {
  free(): void;
/**
*/
  constructor();
/**
* @param {RawIslandManager} islands
* @param {RawRigidBodySet} bodies
* @param {RawColliderSet} colliders
*/
  update(islands: RawIslandManager, bodies: RawRigidBodySet, colliders: RawColliderSet): void;
/**
* @param {RawColliderSet} colliders
* @param {RawVector} rayOrig
* @param {RawVector} rayDir
* @param {number} maxToi
* @param {boolean} solid
* @param {number} groups
* @returns {RawRayColliderToi | undefined}
*/
  castRay(colliders: RawColliderSet, rayOrig: RawVector, rayDir: RawVector, maxToi: number, solid: boolean, groups: number): RawRayColliderToi | undefined;
/**
* @param {RawColliderSet} colliders
* @param {RawVector} rayOrig
* @param {RawVector} rayDir
* @param {number} maxToi
* @param {boolean} solid
* @param {number} groups
* @returns {RawRayColliderIntersection | undefined}
*/
  castRayAndGetNormal(colliders: RawColliderSet, rayOrig: RawVector, rayDir: RawVector, maxToi: number, solid: boolean, groups: number): RawRayColliderIntersection | undefined;
/**
* @param {RawColliderSet} colliders
* @param {RawVector} rayOrig
* @param {RawVector} rayDir
* @param {number} maxToi
* @param {boolean} solid
* @param {number} groups
* @param {Function} callback
*/
  intersectionsWithRay(colliders: RawColliderSet, rayOrig: RawVector, rayDir: RawVector, maxToi: number, solid: boolean, groups: number, callback: Function): void;
/**
* @param {RawColliderSet} colliders
* @param {RawVector} shapePos
* @param {RawRotation} shapeRot
* @param {RawShape} shape
* @param {number} groups
* @returns {number | undefined}
*/
  intersectionWithShape(colliders: RawColliderSet, shapePos: RawVector, shapeRot: RawRotation, shape: RawShape, groups: number): number | undefined;
/**
* @param {RawColliderSet} colliders
* @param {RawVector} point
* @param {boolean} solid
* @param {number} groups
* @returns {RawPointColliderProjection | undefined}
*/
  projectPoint(colliders: RawColliderSet, point: RawVector, solid: boolean, groups: number): RawPointColliderProjection | undefined;
/**
* @param {RawColliderSet} colliders
* @param {RawVector} point
* @param {number} groups
* @param {Function} callback
*/
  intersectionsWithPoint(colliders: RawColliderSet, point: RawVector, groups: number, callback: Function): void;
/**
* @param {RawColliderSet} colliders
* @param {RawVector} shapePos
* @param {RawRotation} shapeRot
* @param {RawVector} shapeVel
* @param {RawShape} shape
* @param {number} maxToi
* @param {number} groups
* @returns {RawShapeColliderTOI | undefined}
*/
  castShape(colliders: RawColliderSet, shapePos: RawVector, shapeRot: RawRotation, shapeVel: RawVector, shape: RawShape, maxToi: number, groups: number): RawShapeColliderTOI | undefined;
/**
* @param {RawColliderSet} colliders
* @param {RawVector} shapePos
* @param {RawRotation} shapeRot
* @param {RawShape} shape
* @param {number} groups
* @param {Function} callback
*/
  intersectionsWithShape(colliders: RawColliderSet, shapePos: RawVector, shapeRot: RawRotation, shape: RawShape, groups: number, callback: Function): void;
/**
* @param {RawVector} aabbCenter
* @param {RawVector} aabbHalfExtents
* @param {Function} callback
*/
  collidersWithAabbIntersectingAabb(aabbCenter: RawVector, aabbHalfExtents: RawVector, callback: Function): void;
}
/**
*/
export class RawRayColliderIntersection {
  free(): void;
/**
* @returns {number}
*/
  colliderHandle(): number;
/**
* @returns {RawVector}
*/
  normal(): RawVector;
/**
* @returns {number}
*/
  toi(): number;
}
/**
*/
export class RawRayColliderToi {
  free(): void;
/**
* @returns {number}
*/
  colliderHandle(): number;
/**
* @returns {number}
*/
  toi(): number;
}
/**
*/
export class RawRigidBodySet {
  free(): void;
/**
* The world-space translation of this rigid-body.
* @param {number} handle
* @returns {RawVector}
*/
  rbTranslation(handle: number): RawVector;
/**
* The world-space orientation of this rigid-body.
* @param {number} handle
* @returns {RawRotation}
*/
  rbRotation(handle: number): RawRotation;
/**
* Put the given rigid-body to sleep.
* @param {number} handle
*/
  rbSleep(handle: number): void;
/**
* Is this rigid-body sleeping?
* @param {number} handle
* @returns {boolean}
*/
  rbIsSleeping(handle: number): boolean;
/**
* Is the velocity of this rigid-body not zero?
* @param {number} handle
* @returns {boolean}
*/
  rbIsMoving(handle: number): boolean;
/**
* The world-space predicted translation of this rigid-body.
*
* If this rigid-body is kinematic this value is set by the `setNextKinematicTranslation`
* method and is used for estimating the kinematic body velocity at the next timestep.
* For non-kinematic bodies, this value is currently unspecified.
* @param {number} handle
* @returns {RawVector}
*/
  rbNextTranslation(handle: number): RawVector;
/**
* The world-space predicted orientation of this rigid-body.
*
* If this rigid-body is kinematic this value is set by the `setNextKinematicRotation`
* method and is used for estimating the kinematic body velocity at the next timestep.
* For non-kinematic bodies, this value is currently unspecified.
* @param {number} handle
* @returns {RawRotation}
*/
  rbNextRotation(handle: number): RawRotation;
/**
* Sets the translation of this rigid-body.
*
* # Parameters
* - `x`: the world-space position of the rigid-body along the `x` axis.
* - `y`: the world-space position of the rigid-body along the `y` axis.
* - `z`: the world-space position of the rigid-body along the `z` axis.
* - `wakeUp`: forces the rigid-body to wake-up so it is properly affected by forces if it
* wasn't moving before modifying its position.
* @param {number} handle
* @param {number} x
* @param {number} y
* @param {number} z
* @param {boolean} wakeUp
*/
  rbSetTranslation(handle: number, x: number, y: number, z: number, wakeUp: boolean): void;
/**
* Sets the rotation quaternion of this rigid-body.
*
* This does nothing if a zero quaternion is provided.
*
* # Parameters
* - `x`: the first vector component of the quaternion.
* - `y`: the second vector component of the quaternion.
* - `z`: the third vector component of the quaternion.
* - `w`: the scalar component of the quaternion.
* - `wakeUp`: forces the rigid-body to wake-up so it is properly affected by forces if it
* wasn't moving before modifying its position.
* @param {number} handle
* @param {number} x
* @param {number} y
* @param {number} z
* @param {number} w
* @param {boolean} wakeUp
*/
  rbSetRotation(handle: number, x: number, y: number, z: number, w: number, wakeUp: boolean): void;
/**
* Sets the linear velocity of this rigid-body.
* @param {number} handle
* @param {RawVector} linvel
* @param {boolean} wakeUp
*/
  rbSetLinvel(handle: number, linvel: RawVector, wakeUp: boolean): void;
/**
* Sets the angular velocity of this rigid-body.
* @param {number} handle
* @param {RawVector} angvel
* @param {boolean} wakeUp
*/
  rbSetAngvel(handle: number, angvel: RawVector, wakeUp: boolean): void;
/**
* If this rigid body is kinematic, sets its future translation after the next timestep integration.
*
* This should be used instead of `rigidBody.setTranslation` to make the dynamic object
* interacting with this kinematic body behave as expected. Internally, Rapier will compute
* an artificial velocity for this rigid-body from its current position and its next kinematic
* position. This velocity will be used to compute forces on dynamic bodies interacting with
* this body.
*
* # Parameters
* - `x`: the world-space position of the rigid-body along the `x` axis.
* - `y`: the world-space position of the rigid-body along the `y` axis.
* - `z`: the world-space position of the rigid-body along the `z` axis.
* @param {number} handle
* @param {number} x
* @param {number} y
* @param {number} z
*/
  rbSetNextKinematicTranslation(handle: number, x: number, y: number, z: number): void;
/**
* If this rigid body is kinematic, sets its future rotation after the next timestep integration.
*
* This should be used instead of `rigidBody.setRotation` to make the dynamic object
* interacting with this kinematic body behave as expected. Internally, Rapier will compute
* an artificial velocity for this rigid-body from its current position and its next kinematic
* position. This velocity will be used to compute forces on dynamic bodies interacting with
* this body.
*
* # Parameters
* - `x`: the first vector component of the quaternion.
* - `y`: the second vector component of the quaternion.
* - `z`: the third vector component of the quaternion.
* - `w`: the scalar component of the quaternion.
* @param {number} handle
* @param {number} x
* @param {number} y
* @param {number} z
* @param {number} w
*/
  rbSetNextKinematicRotation(handle: number, x: number, y: number, z: number, w: number): void;
/**
* The linear velocity of this rigid-body.
* @param {number} handle
* @returns {RawVector}
*/
  rbLinvel(handle: number): RawVector;
/**
* The angular velocity of this rigid-body.
* @param {number} handle
* @returns {RawVector}
*/
  rbAngvel(handle: number): RawVector;
/**
* @param {number} handle
* @param {boolean} locked
* @param {boolean} wake_up
*/
  rbLockTranslations(handle: number, locked: boolean, wake_up: boolean): void;
/**
* @param {number} handle
* @param {boolean} allow_x
* @param {boolean} allow_y
* @param {boolean} allow_z
* @param {boolean} wake_up
*/
  rbRestrictTranslations(handle: number, allow_x: boolean, allow_y: boolean, allow_z: boolean, wake_up: boolean): void;
/**
* @param {number} handle
* @param {boolean} locked
* @param {boolean} wake_up
*/
  rbLockRotations(handle: number, locked: boolean, wake_up: boolean): void;
/**
* @param {number} handle
* @param {boolean} allow_x
* @param {boolean} allow_y
* @param {boolean} allow_z
* @param {boolean} wake_up
*/
  rbRestrictRotations(handle: number, allow_x: boolean, allow_y: boolean, allow_z: boolean, wake_up: boolean): void;
/**
* @param {number} handle
* @returns {number}
*/
  rbDominanceGroup(handle: number): number;
/**
* @param {number} handle
* @param {number} group
*/
  rbSetDominanceGroup(handle: number, group: number): void;
/**
* @param {number} handle
* @param {boolean} enabled
*/
  rbEnableCcd(handle: number, enabled: boolean): void;
/**
* The mass of this rigid-body.
* @param {number} handle
* @returns {number}
*/
  rbMass(handle: number): number;
/**
* Wakes this rigid-body up.
*
* A dynamic rigid-body that does not move during several consecutive frames will
* be put to sleep by the physics engine, i.e., it will stop being simulated in order
* to avoid useless computations.
* This methods forces a sleeping rigid-body to wake-up. This is useful, e.g., before modifying
* the position of a dynamic body so that it is properly simulated afterwards.
* @param {number} handle
*/
  rbWakeUp(handle: number): void;
/**
* Is Continuous Collision Detection enabled for this rigid-body?
* @param {number} handle
* @returns {boolean}
*/
  rbIsCcdEnabled(handle: number): boolean;
/**
* The number of colliders attached to this rigid-body.
* @param {number} handle
* @returns {number}
*/
  rbNumColliders(handle: number): number;
/**
* Retrieves the `i-th` collider attached to this rigid-body.
*
* # Parameters
* - `at`: The index of the collider to retrieve. Must be a number in `[0, this.numColliders()[`.
*         This index is **not** the same as the unique identifier of the collider.
* @param {number} handle
* @param {number} at
* @returns {number}
*/
  rbCollider(handle: number, at: number): number;
/**
* The status of this rigid-body: static, dynamic, or kinematic.
* @param {number} handle
* @returns {number}
*/
  rbBodyType(handle: number): number;
/**
* Is this rigid-body static?
* @param {number} handle
* @returns {boolean}
*/
  rbIsStatic(handle: number): boolean;
/**
* Is this rigid-body kinematic?
* @param {number} handle
* @returns {boolean}
*/
  rbIsKinematic(handle: number): boolean;
/**
* Is this rigid-body dynamic?
* @param {number} handle
* @returns {boolean}
*/
  rbIsDynamic(handle: number): boolean;
/**
* The linear damping coefficient of this rigid-body.
* @param {number} handle
* @returns {number}
*/
  rbLinearDamping(handle: number): number;
/**
* The angular damping coefficient of this rigid-body.
* @param {number} handle
* @returns {number}
*/
  rbAngularDamping(handle: number): number;
/**
* @param {number} handle
* @param {number} factor
*/
  rbSetLinearDamping(handle: number, factor: number): void;
/**
* @param {number} handle
* @param {number} factor
*/
  rbSetAngularDamping(handle: number, factor: number): void;
/**
* @param {number} handle
* @returns {number}
*/
  rbGravityScale(handle: number): number;
/**
* @param {number} handle
* @param {number} factor
* @param {boolean} wakeUp
*/
  rbSetGravityScale(handle: number, factor: number, wakeUp: boolean): void;
/**
* Applies a force at the center-of-mass of this rigid-body.
*
* # Parameters
* - `force`: the world-space force to apply on the rigid-body.
* - `wakeUp`: should the rigid-body be automatically woken-up?
* @param {number} handle
* @param {RawVector} force
* @param {boolean} wakeUp
*/
  rbApplyForce(handle: number, force: RawVector, wakeUp: boolean): void;
/**
* Applies an impulse at the center-of-mass of this rigid-body.
*
* # Parameters
* - `impulse`: the world-space impulse to apply on the rigid-body.
* - `wakeUp`: should the rigid-body be automatically woken-up?
* @param {number} handle
* @param {RawVector} impulse
* @param {boolean} wakeUp
*/
  rbApplyImpulse(handle: number, impulse: RawVector, wakeUp: boolean): void;
/**
* Applies a torque at the center-of-mass of this rigid-body.
*
* # Parameters
* - `torque`: the world-space torque to apply on the rigid-body.
* - `wakeUp`: should the rigid-body be automatically woken-up?
* @param {number} handle
* @param {RawVector} torque
* @param {boolean} wakeUp
*/
  rbApplyTorque(handle: number, torque: RawVector, wakeUp: boolean): void;
/**
* Applies an impulsive torque at the center-of-mass of this rigid-body.
*
* # Parameters
* - `torque impulse`: the world-space torque impulse to apply on the rigid-body.
* - `wakeUp`: should the rigid-body be automatically woken-up?
* @param {number} handle
* @param {RawVector} torque_impulse
* @param {boolean} wakeUp
*/
  rbApplyTorqueImpulse(handle: number, torque_impulse: RawVector, wakeUp: boolean): void;
/**
* Applies a force at the given world-space point of this rigid-body.
*
* # Parameters
* - `force`: the world-space force to apply on the rigid-body.
* - `point`: the world-space point where the impulse is to be applied on the rigid-body.
* - `wakeUp`: should the rigid-body be automatically woken-up?
* @param {number} handle
* @param {RawVector} force
* @param {RawVector} point
* @param {boolean} wakeUp
*/
  rbApplyForceAtPoint(handle: number, force: RawVector, point: RawVector, wakeUp: boolean): void;
/**
* Applies an impulse at the given world-space point of this rigid-body.
*
* # Parameters
* - `impulse`: the world-space impulse to apply on the rigid-body.
* - `point`: the world-space point where the impulse is to be applied on the rigid-body.
* - `wakeUp`: should the rigid-body be automatically woken-up?
* @param {number} handle
* @param {RawVector} impulse
* @param {RawVector} point
* @param {boolean} wakeUp
*/
  rbApplyImpulseAtPoint(handle: number, impulse: RawVector, point: RawVector, wakeUp: boolean): void;
/**
*/
  constructor();
/**
* @param {RawVector} translation
* @param {RawRotation} rotation
* @param {number} gravityScale
* @param {number} mass
* @param {RawVector} centerOfMass
* @param {RawVector} linvel
* @param {RawVector} angvel
* @param {RawVector} principalAngularInertia
* @param {RawRotation} angularInertiaFrame
* @param {boolean} translationEnabledX
* @param {boolean} translationEnabledY
* @param {boolean} translationEnabledZ
* @param {boolean} rotationEnabledX
* @param {boolean} rotationEnabledY
* @param {boolean} rotationEnabledZ
* @param {number} linearDamping
* @param {number} angularDamping
* @param {number} rb_type
* @param {boolean} canSleep
* @param {boolean} ccdEnabled
* @param {number} dominanceGroup
* @returns {number}
*/
  createRigidBody(translation: RawVector, rotation: RawRotation, gravityScale: number, mass: number, centerOfMass: RawVector, linvel: RawVector, angvel: RawVector, principalAngularInertia: RawVector, angularInertiaFrame: RawRotation, translationEnabledX: boolean, translationEnabledY: boolean, translationEnabledZ: boolean, rotationEnabledX: boolean, rotationEnabledY: boolean, rotationEnabledZ: boolean, linearDamping: number, angularDamping: number, rb_type: number, canSleep: boolean, ccdEnabled: boolean, dominanceGroup: number): number;
/**
* @param {number} handle
* @param {RawIslandManager} islands
* @param {RawColliderSet} colliders
* @param {RawImpulseJointSet} joints
* @param {RawMultibodyJointSet} articulations
*/
  remove(handle: number, islands: RawIslandManager, colliders: RawColliderSet, joints: RawImpulseJointSet, articulations: RawMultibodyJointSet): void;
/**
* The number of rigid-bodies on this set.
* @returns {number}
*/
  len(): number;
/**
* Checks if a rigid-body with the given integer handle exists.
* @param {number} handle
* @returns {boolean}
*/
  contains(handle: number): boolean;
/**
* Applies the given JavaScript function to the integer handle of each rigid-body managed by this set.
*
* # Parameters
* - `f(handle)`: the function to apply to the integer handle of each rigid-body managed by this set. Called as `f(collider)`.
* @param {Function} f
*/
  forEachRigidBodyHandle(f: Function): void;
}
/**
* A rotation quaternion.
*/
export class RawRotation {
  free(): void;
/**
* @param {number} x
* @param {number} y
* @param {number} z
* @param {number} w
*/
  constructor(x: number, y: number, z: number, w: number);
/**
* The identity quaternion.
* @returns {RawRotation}
*/
  static identity(): RawRotation;
/**
* The `w` component of this quaternion.
* @returns {number}
*/
  readonly w: number;
/**
* The `x` component of this quaternion.
* @returns {number}
*/
  readonly x: number;
/**
* The `y` component of this quaternion.
* @returns {number}
*/
  readonly y: number;
/**
* The `z` component of this quaternion.
* @returns {number}
*/
  readonly z: number;
}
/**
*/
export class RawSerializationPipeline {
  free(): void;
/**
*/
  constructor();
/**
* @param {RawVector} gravity
* @param {RawIntegrationParameters} integrationParameters
* @param {RawIslandManager} islands
* @param {RawBroadPhase} broadPhase
* @param {RawNarrowPhase} narrowPhase
* @param {RawRigidBodySet} bodies
* @param {RawColliderSet} colliders
* @param {RawImpulseJointSet} impulse_joints
* @param {RawMultibodyJointSet} multibody_joints
* @returns {Uint8Array | undefined}
*/
  serializeAll(gravity: RawVector, integrationParameters: RawIntegrationParameters, islands: RawIslandManager, broadPhase: RawBroadPhase, narrowPhase: RawNarrowPhase, bodies: RawRigidBodySet, colliders: RawColliderSet, impulse_joints: RawImpulseJointSet, multibody_joints: RawMultibodyJointSet): Uint8Array | undefined;
/**
* @param {Uint8Array} data
* @returns {RawDeserializedWorld | undefined}
*/
  deserializeAll(data: Uint8Array): RawDeserializedWorld | undefined;
}
/**
*/
export class RawShape {
  free(): void;
/**
* @param {number} hx
* @param {number} hy
* @param {number} hz
* @returns {RawShape}
*/
  static cuboid(hx: number, hy: number, hz: number): RawShape;
/**
* @param {number} hx
* @param {number} hy
* @param {number} hz
* @param {number} borderRadius
* @returns {RawShape}
*/
  static roundCuboid(hx: number, hy: number, hz: number, borderRadius: number): RawShape;
/**
* @param {number} radius
* @returns {RawShape}
*/
  static ball(radius: number): RawShape;
/**
* @param {number} halfHeight
* @param {number} radius
* @returns {RawShape}
*/
  static capsule(halfHeight: number, radius: number): RawShape;
/**
* @param {number} halfHeight
* @param {number} radius
* @returns {RawShape}
*/
  static cylinder(halfHeight: number, radius: number): RawShape;
/**
* @param {number} halfHeight
* @param {number} radius
* @param {number} borderRadius
* @returns {RawShape}
*/
  static roundCylinder(halfHeight: number, radius: number, borderRadius: number): RawShape;
/**
* @param {number} halfHeight
* @param {number} radius
* @returns {RawShape}
*/
  static cone(halfHeight: number, radius: number): RawShape;
/**
* @param {number} halfHeight
* @param {number} radius
* @param {number} borderRadius
* @returns {RawShape}
*/
  static roundCone(halfHeight: number, radius: number, borderRadius: number): RawShape;
/**
* @param {Float32Array} vertices
* @param {Uint32Array} indices
* @returns {RawShape}
*/
  static polyline(vertices: Float32Array, indices: Uint32Array): RawShape;
/**
* @param {Float32Array} vertices
* @param {Uint32Array} indices
* @returns {RawShape}
*/
  static trimesh(vertices: Float32Array, indices: Uint32Array): RawShape;
/**
* @param {number} nrows
* @param {number} ncols
* @param {Float32Array} heights
* @param {RawVector} scale
* @returns {RawShape}
*/
  static heightfield(nrows: number, ncols: number, heights: Float32Array, scale: RawVector): RawShape;
/**
* @param {RawVector} p1
* @param {RawVector} p2
* @returns {RawShape}
*/
  static segment(p1: RawVector, p2: RawVector): RawShape;
/**
* @param {RawVector} p1
* @param {RawVector} p2
* @param {RawVector} p3
* @returns {RawShape}
*/
  static triangle(p1: RawVector, p2: RawVector, p3: RawVector): RawShape;
/**
* @param {RawVector} p1
* @param {RawVector} p2
* @param {RawVector} p3
* @param {number} borderRadius
* @returns {RawShape}
*/
  static roundTriangle(p1: RawVector, p2: RawVector, p3: RawVector, borderRadius: number): RawShape;
/**
* @param {Float32Array} points
* @returns {RawShape | undefined}
*/
  static convexHull(points: Float32Array): RawShape | undefined;
/**
* @param {Float32Array} points
* @param {number} borderRadius
* @returns {RawShape | undefined}
*/
  static roundConvexHull(points: Float32Array, borderRadius: number): RawShape | undefined;
/**
* @param {Float32Array} vertices
* @param {Uint32Array} indices
* @returns {RawShape | undefined}
*/
  static convexMesh(vertices: Float32Array, indices: Uint32Array): RawShape | undefined;
/**
* @param {Float32Array} vertices
* @param {Uint32Array} indices
* @param {number} borderRadius
* @returns {RawShape | undefined}
*/
  static roundConvexMesh(vertices: Float32Array, indices: Uint32Array, borderRadius: number): RawShape | undefined;
}
/**
*/
export class RawShapeColliderTOI {
  free(): void;
/**
* @returns {number}
*/
  colliderHandle(): number;
/**
* @returns {number}
*/
  toi(): number;
/**
* @returns {RawVector}
*/
  witness1(): RawVector;
/**
* @returns {RawVector}
*/
  witness2(): RawVector;
/**
* @returns {RawVector}
*/
  normal1(): RawVector;
/**
* @returns {RawVector}
*/
  normal2(): RawVector;
}
/**
* A vector.
*/
export class RawVector {
  free(): void;
/**
* Creates a new vector filled with zeros.
* @returns {RawVector}
*/
  static zero(): RawVector;
/**
* Creates a new 3D vector from its two components.
*
* # Parameters
* - `x`: the `x` component of this 3D vector.
* - `y`: the `y` component of this 3D vector.
* - `z`: the `z` component of this 3D vector.
* @param {number} x
* @param {number} y
* @param {number} z
*/
  constructor(x: number, y: number, z: number);
/**
* Create a new 3D vector from this vector with its components rearranged as `{x, y, z}`.
*
* This will effectively return a copy of `this`. This method exist for completeness with the
* other swizzling functions.
* @returns {RawVector}
*/
  xyz(): RawVector;
/**
* Create a new 3D vector from this vector with its components rearranged as `{y, x, z}`.
* @returns {RawVector}
*/
  yxz(): RawVector;
/**
* Create a new 3D vector from this vector with its components rearranged as `{z, x, y}`.
* @returns {RawVector}
*/
  zxy(): RawVector;
/**
* Create a new 3D vector from this vector with its components rearranged as `{x, z, y}`.
* @returns {RawVector}
*/
  xzy(): RawVector;
/**
* Create a new 3D vector from this vector with its components rearranged as `{y, z, x}`.
* @returns {RawVector}
*/
  yzx(): RawVector;
/**
* Create a new 3D vector from this vector with its components rearranged as `{z, y, x}`.
* @returns {RawVector}
*/
  zyx(): RawVector;
/**
* The `x` component of this vector.
* @returns {number}
*/
  x: number;
/**
* The `y` component of this vector.
* @returns {number}
*/
  y: number;
/**
* The `z` component of this vector.
* @returns {number}
*/
  z: number;
}
