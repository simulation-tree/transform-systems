using Collections.Generic;
using Simulation;
using System;
using System.Numerics;
using Transforms.Components;
using Unmanaged;
using Worlds;

namespace Transforms.Systems
{
    public readonly partial struct TransformSystem : ISystem
    {
        private readonly Array<uint> parentEntities;
        private readonly Array<Vector3> pivots;
        private readonly Array<Anchor> anchors;
        private readonly Array<bool> hasAnchors;
        private readonly Array<LocalToWorld> ltwValues;
        private readonly Array<LocalToWorld> anchoredLtwValues;
        private readonly Array<Quaternion> worldRotations;
        private readonly List<List<uint>> sortedEntities;
        private readonly Operation operation;
        private readonly Array<Vector3> positions;
        private readonly Array<Vector3> scales;
        private readonly Array<Quaternion> rotations;
        private readonly Array<Quaternion> eulerAngles;

        public TransformSystem()
        {
            parentEntities = new();
            pivots = new();
            anchors = new();
            hasAnchors = new();
            ltwValues = new();
            anchoredLtwValues = new();
            worldRotations = new();
            sortedEntities = new();
            operation = new();
            positions = new();
            scales = new();
            rotations = new();
            eulerAngles = new();
        }

        public readonly void Dispose()
        {
            foreach (List<uint> entities in sortedEntities)
            {
                entities.Dispose();
            }

            eulerAngles.Dispose();
            rotations.Dispose();
            scales.Dispose();
            positions.Dispose();
            operation.Dispose();
            sortedEntities.Dispose();
            worldRotations.Dispose();
            ltwValues.Dispose();
            anchoredLtwValues.Dispose();
            parentEntities.Dispose();
            anchors.Dispose();
            hasAnchors.Dispose();
            pivots.Dispose();
        }

        void ISystem.Start(in SystemContext context, in World world)
        {
        }

        void ISystem.Update(in SystemContext context, in World world, in TimeSpan delta)
        {
            Update(world);
        }

        void ISystem.Finish(in SystemContext context, in World world)
        {
        }

        private readonly void Update(World world)
        {
            int positionComponent = world.Schema.GetComponentType<Position>();
            int scaleComponent = world.Schema.GetComponentType<Scale>();
            int rotationComponent = world.Schema.GetComponentType<Rotation>();
            int eulerAnglesComponent = world.Schema.GetComponentType<EulerAngles>();
            int anchorComponent = world.Schema.GetComponentType<Anchor>();
            int pivotComponent = world.Schema.GetComponentType<Pivot>();
            int ltwComponent = world.Schema.GetComponentType<LocalToWorld>();
            int worldRotationComponent = world.Schema.GetComponentType<WorldRotation>();
            int transformTag = world.Schema.GetTagType<IsTransform>();

            //ensure capacity is met
            int capacity = (world.MaxEntityValue + 1).GetNextPowerOf2();
            if (ltwValues.Length < capacity)
            {
                parentEntities.Length = capacity;
                ltwValues.Length = capacity;
                worldRotations.Length = capacity;
                anchoredLtwValues.Length = capacity;
                pivots.Length = capacity;
                anchors.Length = capacity;
                hasAnchors.Length = capacity;
                positions.Length = capacity;
                scales.Length = capacity;
                rotations.Length = capacity;
                eulerAngles.Length = capacity;
            }

            //clear state
            parentEntities.Clear();
            ltwValues.Fill(LocalToWorld.Default);
            worldRotations.Fill(Rotation.Default.value);
            anchoredLtwValues.Clear();
            pivots.Clear();
            hasAnchors.Clear();
            anchors.Clear();
            positions.Clear();
            scales.Fill(Scale.Default.value);
            rotations.Fill(Rotation.Default.value);
            eulerAngles.Fill(Rotation.Default.value);

            //reset entities list
            foreach (List<uint> entities in sortedEntities)
            {
                entities.Clear();
            }

            FindComponents(world, positionComponent, rotationComponent, eulerAnglesComponent, scaleComponent, pivotComponent, anchorComponent, transformTag);
            AddMissingComponents(world, transformTag, ltwComponent, worldRotationComponent);
            CalculateTransforms();

            //apply values
            ApplyValues(world, ltwComponent, worldRotationComponent, transformTag);
        }

        private void CalculateTransforms()
        {
            Span<uint> parentEntities = this.parentEntities.AsSpan();
            Span<LocalToWorld> ltwValues = this.ltwValues.AsSpan();
            Span<Quaternion> worldRotations = this.worldRotations.AsSpan();
            Span<Anchor> anchors = this.anchors.AsSpan();
            Span<Vector3> pivots = this.pivots.AsSpan();
            Span<bool> hasAnchors = this.hasAnchors.AsSpan();

            //calculate ltw in descending order (roots towards leafs)
            //where each entity list is descending in depth
            foreach (List<uint> entities in sortedEntities)
            {
                foreach (uint entity in entities)
                {
                    uint parent = parentEntities[(int)entity];
                    ref LocalToWorld ltw = ref ltwValues[(int)entity];
                    ref Quaternion worldRotation = ref worldRotations[(int)entity];
                    if (parent != default)
                    {
                        LocalToWorld parentLtw = ltwValues[(int)parent];
                        Quaternion parentWorldRotation = worldRotations[(int)parent];
                        if (hasAnchors[(int)entity])
                        {
                            Anchor anchor = anchors[(int)entity];
                            (Vector3 parentPosition, Quaternion _, Vector3 parentSize) = parentLtw.Decomposed;
                            float minX = anchor.minX.Number;
                            float minY = anchor.minY.Number;
                            float minZ = anchor.minZ.Number;
                            float maxX = anchor.maxX.Number;
                            float maxY = anchor.maxY.Number;
                            float maxZ = anchor.maxZ.Number;
                            if (!anchor.minX.Absolute)
                            {
                                minX *= parentSize.X;
                            }

                            if (!anchor.minY.Absolute)
                            {
                                minY *= parentSize.Y;
                            }

                            if (!anchor.minZ.Absolute)
                            {
                                minZ *= parentSize.Z;
                            }

                            if (!anchor.maxX.Absolute)
                            {
                                maxX *= parentSize.X;
                            }
                            else
                            {
                                maxX = parentSize.X - maxX;
                            }

                            if (!anchor.maxY.Absolute)
                            {
                                maxY *= parentSize.Y;
                            }
                            else
                            {
                                maxY = parentSize.Y - maxY;
                            }

                            if (!anchor.maxZ.Absolute)
                            {
                                maxZ *= parentSize.Z;
                            }
                            else
                            {
                                maxZ = parentSize.Z - maxZ;
                            }

                            Vector3 anchorScale = new(maxX - minX, maxY - minY, maxZ - minZ);
                            Vector3 anchorPosition = parentPosition + new Vector3(minX, minY, minZ);
                            if (anchorScale.X == default)
                            {
                                anchorScale.X = 1f;
                            }

                            if (anchorScale.Y == default)
                            {
                                anchorScale.Y = 1f;
                            }

                            if (anchorScale.Z == default)
                            {
                                anchorScale.Z = 1f;
                            }

                            Matrix4x4 anchorLtw = Matrix4x4.CreateScale(anchorScale) * Matrix4x4.CreateTranslation(anchorPosition);
                            ltw.value *= anchorLtw;

                            //affect ltw by pivot
                            Vector3 anchorPivot = pivots[(int)entity];
                            Vector3 pivotOffset = anchorPivot * ltw.Scale;
                            ltw.value *= Matrix4x4.CreateTranslation(-pivotOffset);
                        }
                        else
                        {
                            ltw.value *= parentLtw.value;
                        }

                        worldRotation = parentWorldRotation * worldRotation;
                    }
                }

                entities.Clear();
            }
        }

        private readonly void FindComponents(World world, int positionComponent, int rotationComponent, int eulerAnglesComponent, int scaleComponent, int pivotComponent, int anchorComponent, int transformTag)
        {
            Span<Vector3> positions = this.positions.AsSpan();
            Span<Vector3> scales = this.scales.AsSpan();
            Span<Quaternion> rotations = this.rotations.AsSpan();
            Span<Quaternion> eulerAngles = this.eulerAngles.AsSpan();
            Span<Vector3> pivots = this.pivots.AsSpan();
            Span<Anchor> anchors = this.anchors.AsSpan();
            Span<bool> hasAnchors = this.hasAnchors.AsSpan();
            Span<uint> parentEntities = this.parentEntities.AsSpan();
            Span<LocalToWorld> ltwValues = this.ltwValues.AsSpan();
            Span<Quaternion> worldRotations = this.worldRotations.AsSpan();

            foreach (Chunk chunk in world.Chunks)
            {
                BitMask componentTypes = chunk.Definition.componentTypes;
                ReadOnlySpan<uint> entities = chunk.Entities;

                if (componentTypes.Contains(positionComponent))
                {
                    ComponentEnumerator<Position> positionComponents = chunk.GetComponents<Position>(positionComponent);
                    for (int i = 0; i < entities.Length; i++)
                    {
                        uint entity = entities[i];
                        positions[(int)entity] = positionComponents[i].value;
                    }
                }

                if (componentTypes.Contains(scaleComponent))
                {
                    ComponentEnumerator<Scale> scaleComponents = chunk.GetComponents<Scale>(scaleComponent);
                    for (int i = 0; i < entities.Length; i++)
                    {
                        uint entity = entities[i];
                        scales[(int)entity] = scaleComponents[i].value;
                    }
                }

                if (componentTypes.Contains(rotationComponent))
                {
                    ComponentEnumerator<Rotation> rotationComponents = chunk.GetComponents<Rotation>(rotationComponent);
                    for (int i = 0; i < entities.Length; i++)
                    {
                        uint entity = entities[i];
                        rotations[(int)entity] = rotationComponents[i].value;
                    }
                }

                if (componentTypes.Contains(eulerAnglesComponent))
                {
                    ComponentEnumerator<EulerAngles> eulerAngleComponents = chunk.GetComponents<EulerAngles>(eulerAnglesComponent);
                    for (int i = 0; i < entities.Length; i++)
                    {
                        uint entity = entities[i];
                        eulerAngles[(int)entity] = eulerAngleComponents[i].AsQuaternion();
                    }
                }

                if (componentTypes.Contains(pivotComponent))
                {
                    ComponentEnumerator<Pivot> pivotComponents = chunk.GetComponents<Pivot>(pivotComponent);
                    for (int i = 0; i < entities.Length; i++)
                    {
                        uint entity = entities[i];
                        pivots[(int)entity] = pivotComponents[i].value;
                    }
                }

                if (componentTypes.Contains(anchorComponent))
                {
                    ComponentEnumerator<Anchor> anchorComponents = chunk.GetComponents<Anchor>(anchorComponent);
                    for (int i = 0; i < entities.Length; i++)
                    {
                        uint entity = entities[i];
                        anchors[(int)entity] = anchorComponents[i];
                        hasAnchors[(int)entity] = true;
                    }
                }
            }

            foreach (Chunk chunk in world.Chunks)
            {
                if (chunk.Definition.tagTypes.Contains(transformTag))
                {
                    ReadOnlySpan<uint> entities = chunk.Entities;
                    foreach (uint entity in entities)
                    {
                        uint parent = world.GetParent(entity);
                        parentEntities[(int)entity] = parent;

                        //get local to parent matrix first
                        ref Vector3 position = ref positions[(int)entity];
                        ref Quaternion eulerAngle = ref eulerAngles[(int)entity];
                        ref Quaternion rotation = ref rotations[(int)entity];
                        ref Vector3 scale = ref scales[(int)entity];
                        Matrix4x4 ltp = Matrix4x4.Identity;
                        Quaternion localRotation = Quaternion.Identity;
                        ltp *= Matrix4x4.CreateScale(scale);
                        localRotation = eulerAngle * localRotation;
                        localRotation = rotation * localRotation;
                        ltp *= Matrix4x4.CreateFromQuaternion(localRotation);
                        Vector3 pivot = !hasAnchors[(int)entity] ? pivots[(int)entity] : Vector3.Zero;
                        ltp *= Matrix4x4.CreateTranslation(position - pivot * scale);

                        ltwValues[(int)entity] = new(ltp);
                        worldRotations[(int)entity] = localRotation;

                        //calculate how deep the entity is
                        int depth = 0;
                        uint current = parent;
                        while (current != default)
                        {
                            depth++;
                            current = world.GetParent(current);
                        }

                        while (sortedEntities.Count <= depth)
                        {
                            sortedEntities.Add(new());
                        }

                        //put the entity into a list located at the index, where the index is the depth
                        sortedEntities[depth].Add(entity);
                    }
                }
            }
        }

        private readonly void AddMissingComponents(World world, int transformTag, int ltwComponent, int worldRotationComponent)
        {
            //go through all entities without a ltw component, and add it
            foreach (Chunk chunk in world.Chunks)
            {
                Definition definition = chunk.Definition;
                if (definition.ContainsTag(transformTag) && !definition.ContainsComponent(ltwComponent))
                {
                    operation.SelectEntities(chunk.Entities);
                }
            }

            if (operation.Count > 0)
            {
                operation.AddComponent<LocalToWorld>();
                operation.ClearSelection();
            }

            //go through all without a world rotation component, and add that too
            bool selectedAny = false;
            foreach (Chunk chunk in world.Chunks)
            {
                Definition definition = chunk.Definition;
                if (definition.ContainsTag(transformTag) && !definition.ContainsComponent(worldRotationComponent))
                {
                    operation.SelectEntities(chunk.Entities);
                    selectedAny = true;
                }
            }

            if (selectedAny)
            {
                operation.AddComponent<WorldRotation>();
            }

            if (operation.Count > 0)
            {
                operation.Perform(world);
                operation.Reset();
            }
        }

        private readonly void ApplyValues(World world, int ltwComponent, int worldRotationComponent, int transformTag)
        {
            Span<LocalToWorld> ltwValues = this.ltwValues.AsSpan();
            Span<Quaternion> worldRotations = this.worldRotations.AsSpan();
            foreach (Chunk chunk in world.Chunks)
            {
                Definition key = chunk.Definition;
                if (key.ContainsTag(transformTag) && key.ContainsComponent(ltwComponent) && key.ContainsComponent(worldRotationComponent))
                {
                    ReadOnlySpan<uint> entities = chunk.Entities;
                    ComponentEnumerator<LocalToWorld> ltwComponents = chunk.GetComponents<LocalToWorld>(ltwComponent);
                    ComponentEnumerator<WorldRotation> worldRotationComponents = chunk.GetComponents<WorldRotation>(worldRotationComponent);
                    for (int i = 0; i < entities.Length; i++)
                    {
                        uint entity = entities[i];
                        ltwComponents[i] = ltwValues[(int)entity];
                        worldRotationComponents[i] = new(worldRotations[(int)entity]);
                    }
                }
            }
        }
    }
}
