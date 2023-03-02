
import { useGLTF } from "@react-three/drei";
import mesh from "./Torso.glb";

export default function Model() {
  const { nodes, materials } = useGLTF(mesh);

  return [{
    type: 'group', children: [{
      type: 'raw', geometry: nodes.imagetostl_mesh_1.geometry,
      material: materials.mat0
    },

    {
      type: 'raw', geometry: nodes.imagetostl_mesh_2.geometry,
      material: materials["Material.001"]
    },

    {
      type: 'raw', geometry: nodes.imagetostl_mesh_3.geometry,
      material: materials["Material.002"]
    }]
  }]



}

useGLTF.preload(mesh);
