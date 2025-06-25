using Collections.Generic;
using Simulation;
using System;
using System.Numerics;
using Transforms.Components;
using Transforms.Messages;
using Unmanaged;
using Worlds;

namespace Transforms.Systems
{
    public sealed partial class TransformSystem : SystemBase, IListener<TransformUpdate>
    {
        private readonly World world;
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
        private readonly int positionType;
        private readonly int scaleType;
        private readonly int rotationType;
        private readonly int anchorType;
        private readonly int pivotType;
        private readonly int ltwType;
        private readonly int worldRotationType;
        private readonly int tagType;
        private readonly BitMask transformComponents;

        public TransformSystem(Simulator simulator, World world) : base(simulator)
        {
            this.world = world;
            parentEntities = new();
            pivots = new();
            anchors = new();
            hasAnchors = new();
            ltwValues = new();
            anchoredLtwValues = new();
            worldRotations = new();
            sortedEntities = new();
            operation = new(world);
            positions = new();
            scales = new();
            rotations = new();

            Schema schema = world.Schema;
            positionType = schema.GetComponentType<Position>();
            scaleType = schema.GetComponentType<Scale>();
            rotationType = schema.GetComponentType<Rotation>();
            anchorType = schema.GetComponentType<Anchor>();
            pivotType = schema.GetComponentType<Pivot>();
            ltwType = schema.GetComponentType<LocalToWorld>();
            worldRotationType = schema.GetComponentType<WorldRotation>();
            tagType = schema.GetTagType<IsTransform>();
            transformComponents = new(ltwType, worldRotationType);
        }

        public override void Dispose()
        {
            foreach (List<uint> entities in sortedEntities)
            {
                entities.Dispose();
            }

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

        void IListener<TransformUpdate>.Receive(ref TransformUpdate message)
        {
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
            }

            int maxDepth = world.MaxDepth;
            if (sortedEntities.Count <= maxDepth)
            {
                int toAdd = maxDepth - sortedEntities.Count;
                for (int i = 0; i <= toAdd; i++)
                {
                    sortedEntities.Add(new(32));
                }
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

            //reset entities list
            Span<List<uint>> sortedEntitiesSpan = sortedEntities.AsSpan();
            for (int d = 0; d < sortedEntitiesSpan.Length; d++)
            {
                sortedEntitiesSpan[d].Clear();
            }

            FindComponents(sortedEntitiesSpan);
            AddMissingComponents();
            CalculateTransforms(sortedEntitiesSpan);
            ApplyValues();
        }

        private void CalculateTransforms(Span<List<uint>> sortedEntities)
        {
            Span<uint> parentEntities = this.parentEntities.AsSpan();
            Span<LocalToWorld> ltwValues = this.ltwValues.AsSpan();
            Span<Quaternion> worldRotations = this.worldRotations.AsSpan();
            Span<Anchor> anchors = this.anchors.AsSpan();
            Span<Vector3> pivots = this.pivots.AsSpan();
            Span<bool> hasAnchors = this.hasAnchors.AsSpan();

            //calculate ltw in descending order (roots towards leafs)
            //where each entity list is descending in depth
            for (int d = 0; d < sortedEntities.Length; d++)
            {
                Span<uint> entitiesSpan = sortedEntities[d].AsSpan();
                for (int e = 0; e < entitiesSpan.Length; e++)
                {
                    uint entity = entitiesSpan[e];
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
                            float minX = anchor.minX;
                            float minY = anchor.minY;
                            float minZ = anchor.minZ;
                            float maxX = anchor.maxX;
                            float maxY = anchor.maxY;
                            float maxZ = anchor.maxZ;
                            Anchor.Relativeness flags = anchor.flags;
                            if ((flags & Anchor.Relativeness.MinX) == 0)
                            {
                                minX *= parentSize.X;
                            }

                            if ((flags & Anchor.Relativeness.MinY) == 0)
                            {
                                minY *= parentSize.Y;
                            }

                            if ((flags & Anchor.Relativeness.MinZ) == 0)
                            {
                                minZ *= parentSize.Z;
                            }

                            if ((flags & Anchor.Relativeness.MaxX) == 0)
                            {
                                maxX *= parentSize.X;
                            }
                            else
                            {
                                maxX = parentSize.X - maxX;
                            }

                            if ((flags & Anchor.Relativeness.MaxY) == 0)
                            {
                                maxY *= parentSize.Y;
                            }
                            else
                            {
                                maxY = parentSize.Y - maxY;
                            }

                            if ((flags & Anchor.Relativeness.MaxZ) == 0)
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
            }
        }

        private void FindComponents(Span<List<uint>> sortedEntities)
        {
            Span<Vector3> positions = this.positions.AsSpan();
            Span<Vector3> scales = this.scales.AsSpan();
            Span<Quaternion> rotations = this.rotations.AsSpan();
            Span<Vector3> pivots = this.pivots.AsSpan();
            Span<Anchor> anchors = this.anchors.AsSpan();
            Span<bool> hasAnchors = this.hasAnchors.AsSpan();
            Span<uint> parentEntities = this.parentEntities.AsSpan();
            Span<LocalToWorld> ltwValues = this.ltwValues.AsSpan();
            Span<Quaternion> worldRotations = this.worldRotations.AsSpan();
            ReadOnlySpan<Chunk> chunks = world.Chunks;
            for (int c = 0; c < chunks.Length; c++)
            {
                Chunk chunk = chunks[c];
                Definition definition = chunk.Definition;
                ReadOnlySpan<uint> entities = chunk.Entities;
                BitMask componentTypes = definition.componentTypes;
                if (componentTypes.Contains(positionType))
                {
                    ComponentEnumerator<Position> positionComponents = chunk.GetComponents<Position>(positionType);
                    for (int i = 0; i < entities.Length; i++)
                    {
                        uint entity = entities[i];
                        positions[(int)entity] = positionComponents[i].value;
                    }
                }

                if (componentTypes.Contains(scaleType))
                {
                    ComponentEnumerator<Scale> scaleComponents = chunk.GetComponents<Scale>(scaleType);
                    for (int i = 0; i < entities.Length; i++)
                    {
                        uint entity = entities[i];
                        scales[(int)entity] = scaleComponents[i].value;
                    }
                }

                if (componentTypes.Contains(rotationType))
                {
                    ComponentEnumerator<Rotation> rotationComponents = chunk.GetComponents<Rotation>(rotationType);
                    for (int i = 0; i < entities.Length; i++)
                    {
                        uint entity = entities[i];
                        rotations[(int)entity] = rotationComponents[i].value;
                    }
                }

                if (componentTypes.Contains(pivotType))
                {
                    ComponentEnumerator<Pivot> pivotComponents = chunk.GetComponents<Pivot>(pivotType);
                    for (int i = 0; i < entities.Length; i++)
                    {
                        uint entity = entities[i];
                        pivots[(int)entity] = pivotComponents[i].value;
                    }
                }

                if (componentTypes.Contains(anchorType))
                {
                    ComponentEnumerator<Anchor> anchorComponents = chunk.GetComponents<Anchor>(anchorType);
                    for (int e = 0; e < entities.Length; e++)
                    {
                        uint entity = entities[e];
                        anchors[(int)entity] = anchorComponents[e];
                        hasAnchors[(int)entity] = true;
                    }
                }
            }

            ReadOnlySpan<Slot> slots = world.Slots;
            for (int c = 0; c < chunks.Length; c++)
            {
                Chunk chunk = chunks[c];
                if (chunk.Definition.tagTypes.Contains(tagType))
                {
                    ReadOnlySpan<uint> entities = chunk.Entities;
                    for (int e = 0; e < entities.Length; e++)
                    {
                        uint entity = entities[e];
                        Slot slot = slots[(int)entity];
                        parentEntities[(int)entity] = slot.parent;

                        //get local to parent matrix first
                        ref Vector3 position = ref positions[(int)entity];
                        ref Quaternion rotation = ref rotations[(int)entity];
                        ref Vector3 scale = ref scales[(int)entity];
                        Matrix4x4 ltp = Matrix4x4.Identity;
                        ltp *= Matrix4x4.CreateScale(scale);
                        ltp *= Matrix4x4.CreateFromQuaternion(rotation);
                        Vector3 pivot = !hasAnchors[(int)entity] ? pivots[(int)entity] : Vector3.Zero;
                        ltp *= Matrix4x4.CreateTranslation(position - pivot * scale);

                        ltwValues[(int)entity] = new(ltp);
                        worldRotations[(int)entity] = rotation;

                        //put the entity into a list located at the index, where the index is the depth
                        sortedEntities[slot.depth].Add(entity);
                    }
                }
            }
        }

        private void AddMissingComponents()
        {
            //go through all entities without an ltw or world rotation component, and add them
            ReadOnlySpan<Chunk> chunks = world.Chunks;
            for (int c = 0; c < chunks.Length; c++)
            {
                Chunk chunk = chunks[c];
                ReadOnlySpan<uint> entities = chunk.Entities;
                if (entities.Length > 0)
                {
                    Definition definition = chunk.Definition;
                    if (definition.ContainsTag(tagType))
                    {
                        if (!definition.ContainsComponent(ltwType) || !definition.ContainsComponent(worldRotationType))
                        {
                            operation.AppendMultipleEntitiesToSelection(entities);
                        }
                    }
                }
            }

            if (operation.Count > 0)
            {
                operation.TryAddComponentType(ltwType);
                operation.TryAddComponentType(worldRotationType);
                operation.Perform();
                operation.Reset();
            }
        }

        private void ApplyValues()
        {
            Span<LocalToWorld> ltwValues = this.ltwValues.AsSpan();
            Span<Quaternion> worldRotations = this.worldRotations.AsSpan();
            ReadOnlySpan<Chunk> chunks = world.Chunks;
            for (int c = 0; c < chunks.Length; c++)
            {
                Chunk chunk = chunks[c];
                Definition definition = chunk.Definition;
                if (definition.tagTypes.Contains(tagType) && definition.componentTypes.ContainsAll(transformComponents))
                {
                    ReadOnlySpan<uint> entities = chunk.Entities;
                    ComponentEnumerator<LocalToWorld> ltwComponents = chunk.GetComponents<LocalToWorld>(ltwType);
                    ComponentEnumerator<WorldRotation> worldRotationComponents = chunk.GetComponents<WorldRotation>(worldRotationType);
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