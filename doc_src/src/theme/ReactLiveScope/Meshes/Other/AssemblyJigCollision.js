import { useGLTF } from '@react-three/drei'
import AssemblyJigCollisionFile from './AssemblyJigCollision.glb';

export default function Model(props) {
  const { nodes } = useGLTF(AssemblyJigCollisionFile);
  return [
      { type: 'group', rotation: [Math.PI/2, 0, 0], children: [
        {
            type:'raw',
            geometry:nodes.Mesh_0.geometry,
            material:nodes.Mesh_0.material,
            scale:[5,5,5]
          }
      ]}
    ]
}

useGLTF.preload(AssemblyJigCollisionFile)