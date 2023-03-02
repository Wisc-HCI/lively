import { useGLTF } from '@react-three/drei'
import ConveyorDispatcherCollisionFile from './ConveyorDispatcherCollision.glb';

export default function Model(props) {
  const { nodes } = useGLTF(ConveyorDispatcherCollisionFile);
  return [
      { type: 'group', rotation: [-Math.PI / 2, 0, 0], children: [
        {
            type:'raw',
            geometry:nodes.ConveyorDispatcherCollision.geometry,
            material:nodes.ConveyorDispatcherCollision.material
          }
      ]}
    ]
}

useGLTF.preload(ConveyorDispatcherCollisionFile)