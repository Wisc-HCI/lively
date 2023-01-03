import { useGLTF } from '@react-three/drei'
import ConveyorReceiverCollisionFile from './ConveyorReceiverCollision.glb';

export default function Model(props) {
  const { nodes } = useGLTF(ConveyorReceiverCollisionFile);
  return [
      { type: 'group', rotation: [-Math.PI / 2, 0, 0], children: [
        {
            type:'raw',
            geometry:nodes.ConveyorReceiverCollision.geometry,
            material:nodes.ConveyorReceiverCollision.material
          }
      ]}
    ]
}

useGLTF.preload(ConveyorReceiverCollisionFile)