import React, { memo, useEffect } from "react";
import { Scene, useSceneStore } from "robot-scene";
import { mapValues } from "lodash";
import MeshLookupTable from "./Meshes";

export const RobotViewer = ({
  state,
  links = [],
  showCollision = false,
  shapes,
  transformControl,
  onMove
}) => {
  useEffect(() => {
    let items = {};
    links?.forEach((link) => {
      link.visuals.forEach((visual, i) => {
        items[`visual-${link.name}-${i}`] = shape2item(visual, false);
      });
      if (showCollision) {
        link.collisions.forEach((collision, i) => {
          items[`collision-${link.name}-${i}`] = shape2item(collision, true);
        });
      }
    });
   

    
      shapes?.forEach((shape, i) => {
        items[`env-shape-${shape.name}`] = shape2item(shape, false);
      });
    
    
    if (transformControl) {
      //console.log("transformControl", transformControl);
      items[`transform-controller-${transformControl.name}`] = shape2item(
        transformControl,
        false
      );
    }

    let tfs = state2tfs(state);

    useSceneStore.setState({ items, tfs, onMove });
    //useSceneStore.setState(state => console.log(state));

    //useDefaultSceneStore.setState({items,tfs})
  }, [state, links, onMove]);
  //items[`transformControl-${transformControl.name}`] = shape2item(transformControl,false);

  return <SceneWrapper />;
};

const SceneWrapper = memo(() => {
  return (
    <div style={{ height: 500, marginTop: 4, marginBottom: 4 }}>
      <Scene
        displayGrid={true}
        backgroundColor="#1e1e1e"
        planeColor="#141414"
        highlightColor="#bf65d8"
        plane={0}
        fov={50}
        store={useSceneStore}
        meshLookup={MeshLookupTable}
        // onPointerMissed={clearFocus}
        // paused={paused}
      />
    </div>
  );
});

function shape2item(shape, isCollision ) {
  let item = {
    name: shape.name,
    frame: shape.frame,
    position: {
      x: shape.localTransform.translation[0],
      y: shape.localTransform.translation[1],
      z: shape.localTransform.translation[2],
    },
    rotation: {
      w: shape.localTransform.rotation[3],
      x: shape.localTransform.rotation[0],
      y: shape.localTransform.rotation[1],
      z: shape.localTransform.rotation[2],
    },
    color: isCollision
      ? { r: 100, g: 0, b: 0, a: 1 }
      : { r: 100, g: 100, b: 100, a: 1 },
    scale: { x: 1, y: 1, z: 1 },
    wireframe: isCollision
  };

  switch (shape.type) {
    case "Arrow": 
      item.shape = "arrow";
      item.scale = { x: 0.5, y: 0.5, z: 0.5 };
      item.transformMode = "rotate";
      break;
    case "Box":
      item.shape = "cube";
      item.scale = { x: shape.x, y: shape.y, z: shape.z };
      item.transformMode = shape.transformMode;
      break;
    case "Sphere":
      item.shape = "sphere";
      item.scale = {
        x: shape.radius * 2,
        y: shape.radius * 2,
        z: shape.radius * 2,
      };
      item.transformMode = "translate";
      break;
    case "Cylinder":
      item.shape = "cylinder";
      item.scale = {
        x: shape.radius * 2,
        y: shape.radius * 2,
        z: shape.length,
      };
      break;
    case "Capsule":
      item.shape = "capsule";
      item.scale = {
        x: shape.radius * 2,
        y: shape.radius * 2,
        z: shape.length,
      };
      break;
    case "Mesh":
      item.shape = shape.filename;
      item.scale = { x: shape.x, y: shape.y, z: shape.z };
      item.color = undefined;
      break;
    default:
      item.shape = "box";
  }
  return item;
}

function state2tfs(state) {
  let tfs = {};
  // console.log(state.proximity);
  Object.entries(state?.frames || {}).forEach((pair) => {
    tfs[pair[0]] = {
      frame: "world",
      position: {
        x: pair[1].world.translation[0],
        y: pair[1].world.translation[1],
        z: pair[1].world.translation[2],
      },
      rotation: {
        w: pair[1].world.rotation[3],
        x: pair[1].world.rotation[0],
        y: pair[1].world.rotation[1],
        z: pair[1].world.rotation[2],
      },
      scale: {
        x: 1,
        y: 1,
        z: 1,
      },
    };
    // tfs[pair[0]+'-translation'] = {
    //     frame: 'world',
    //     position: {
    //         x: pair[1].world.translation[0],
    //         y: pair[1].world.translation[1],
    //         z: pair[1].world.translation[2]
    //     },
    //     rotation: {
    //         w: 1,
    //         x: 0,
    //         y: 0,
    //         z: 0
    //     },
    //     scale: {
    //         x: 1,
    //         y: 1,
    //         z: 1
    //     }
    // }
  });
  // console.log(tfs)
  return tfs;
}
