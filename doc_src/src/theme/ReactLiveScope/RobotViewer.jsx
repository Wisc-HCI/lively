import React, { memo, useEffect } from "react";
import { Scene, useSceneStore } from "robot-scene";
// import { mapValues } from "lodash";
import { Button } from "./Button";
import MeshLookupTable from "./Meshes";
import { FullScreen, useFullScreenHandle } from "react-full-screen";
import { useControls, Leva } from "leva";
import { useColorMode } from "@docusaurus/theme-common";

const LevaLightMode = {
  colors: {
    elevation1: "#eee",
    elevation2: "#ddd",
    elevation3: "#eee",
    accent1: "#8528a0",
    accent2: "#b44cd2",
    accent3: "#c26edb",
    highlight1: "#333",
    highlight2: "#444",
    highlight3: "#555",
    vivid1: "#ffcc00",
  },
};

const LevaDarkMode = {
  colors: {
    elevation1: "#333",
    elevation2: "#222",
    elevation3: "#333",
    accent1: "#942db1",
    accent2: "#bf65d8",
    accent3: "#cf8be2",
    highlight1: "#777",
    highlight2: "#999",
    highlight3: "#FEFEFE",
    vivid1: "#ffcc00",
  },
};

export const RobotViewer = ({
  state,
  links = [],
  showCollision = false,
  shapes,
  transformControl,
  onMove,
  levaOptions = {},
}) => {
  const { colorMode } = useColorMode();
  const handle = useFullScreenHandle();
  const { _ } = useControls({
    fullScreen: {
      label: "Full Screen",
      value: handle.active,
      onChange: (v) => {
        if (v) {
          handle.enter();
        } else {
          handle.exit();
        }
      },
    },
    ...levaOptions,
  });
  // const {foo} = useControls({foo:'bar'});

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

  return (
    <>
      <FullScreen handle={handle}>
        <Leva
          flat={!handle.active}
          fill={!handle.active}
          hideCopyButton
          titleBar={{ title: "Example Settings" }}
          theme={colorMode === "dark" ? LevaDarkMode : LevaLightMode}
        />
        <SceneWrapper bounded={!handle.active} />
      </FullScreen>
    </>
  );
};

const SceneWrapper = memo(({ bounded }) => {
  return (
    <div
      style={{ height: bounded ? 500 : "100%", marginTop: 0, marginBottom: 4 }}
    >
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

function shape2item(shape, isCollision) {
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
    wireframe: isCollision,
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
      item.shapeParams = {height: shape.length,radius:shape.radius};
      break;
    case "Capsule":
      item.shape = "capsule";
      item.shapeParams = {height: shape.length,radius:shape.radius};
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
