# Transform Systems

Implements the `transforms` project

### Updating

System performs its work when the `TransformUpdate` message is broadcast.
```cs
using Simulator simulator = new();
using World world = new();
simulator.Add(new TransformSystem(simulator, world));

//create sample case
Transform a = new(world, Vector3.Zero, Quaternion.Identity, Vector3.One * 2f);
Transform b = new(world, Vector3.One, Quaternion.Identity, Vector3.One);
b.SetParent(a);

//calculate ltw values
simulator.Broadcast(new TransformUpdate());

//assert
Assert.That(b.WorldPosition, Is.EqualTo(Vector3.One * 2f));
Assert.That(b.WorldRotation, Is.EqualTo(Quaternion.Identity));
Assert.That(b.WorldScale, Is.EqualTo(Vector3.One * 2f));

simulator.Remove<TransformSystem>();
```