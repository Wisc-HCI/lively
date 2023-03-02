import { useGLTF } from '@react-three/drei'
import ConveyorReceiverFile from './ConveyorReceiver.glb';

export default function Model(props) {
  const { nodes, materials } = useGLTF(ConveyorReceiverFile);
  return [
      { type: 'group', children: [
        {
            type:'raw',
            geometry:nodes.ConveyorReceiver.geometry,
            material:materials.ConveyorAddonMaterial
          }
      ]}
    ]
}

useGLTF.preload(ConveyorReceiverFile)