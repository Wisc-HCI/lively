import { useGLTF } from '@react-three/drei'
import ConveyorCollisionFile from './ConveyorCollision.glb';

export default function Model(props) {
  const { nodes } = useGLTF(ConveyorCollisionFile);
  return [
      { type: 'group', rotation: [-Math.PI / 2, 0, 0], children: [
        {
            type:'raw',
            geometry:nodes.ConveyorCollision.geometry,
            material:nodes.ConveyorCollision.material
          }
      ]}
    ]
}

useGLTF.preload(ConveyorCollisionFile)