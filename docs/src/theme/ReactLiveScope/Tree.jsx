import { motion, AnimatePresence } from "framer-motion";
import React, { useState, memo } from "react";
import { useColorMode } from '@docusaurus/theme-common';

export const Tree = memo(({ label, data, topLevel=true }) => {
  const {colorMode} = useColorMode();
  const [open, setOpen] = useState(false);
  const openable =
    typeof data === "object" && data && Object.keys(data).length > 0;
  return (
    <div
      onClick={(e) => {
        setOpen(!open);
        e.stopPropagation();
      }}
      style={{
        backgroundColor: !topLevel ? 'none' : colorMode === 'dark' ? "#dddddd44" : "#22222244",
        borderRadius: 5,
        padding: 6,
        textAlign: "left",
        color: "white"
      }}
    >
      {typeof data === "object" && data ? (
        <Header
          openable={openable}
          label={label}
          open={open}
          extra={`${Array.isArray(data) ? "Array" : "Object"}[${
            Object.keys(data).length
          }]`}
        />
      ) : typeof data === "string" || typeof data === "number" ? (
        <Header label={label} open={open} value={data} extra={typeof data} />
      ) : typeof data === "boolean" ? (
        <Header label={label} open={open} value={data ? 'true' : 'false'} extra={typeof data} />
      ) : data === null ? (
        <Header label={label} open={open} value={"null"} />
      ) : data === undefined ? (
        <Header label={label} open={open} value={"undefined"} />
      ) : (
        <Header label={label} open={open} value={`not handled (${typeof data})`} />
      )}
      {typeof data === "object" && data !== null && data !== undefined && (
        <AnimatePresence>
          {open &&
            <motion.div
              initial="closed"
              animate="open"
              exit="closed"
              style={{
                backgroundColor: colorMode === 'dark' ? "#dddddd44" : "#22222244",
                borderRadius: 5,
                marginTop: openable ? 5 : 0
              }}
              variants={{
                open: { height: "auto" },
                closed: { height: 0, overflow: "hidden" }
              }}
            >
              {Object.keys(data).map((k) => (
                <Tree key={k} label={k} data={data[k]} topLevel={false}/>
              ))}
            </motion.div>
        }
        </AnimatePresence>
      )}
    </div>
  );
})

const Header = ({ open, label, value, extra, openable }) => (
  <span>
    <span
      style={{
        backgroundColor: open && openable ? "#ffffffbb" : "#30303060",
        boxShadow: `inset 0px 0px 0px 2px ${
          open && openable ? "#b44cd2" : "#444"
        }`,
        borderRadius: 5,
        padding: 4,
        color: open && openable ? "#b44cd2" : "white"
      }}
    >
      {openable && <ExpandCarrot expanded={open} />}
      {label}
    </span>
    {value !== undefined && (
      <span
        style={{
          paddingTop: 2,
          paddingBottom: 2,
          paddingRight: 5,
          paddingLeft: 5,
          color: "white",
          marginLeft: 4
        }}
      >
        {value}
      </span>
    )}
    {extra !== undefined && (
      <span
        style={{
          opacity: 0.35,
          borderRadius: 100,
          backgroundColor: "#303030",
          paddingTop: 2,
          paddingBottom: 2,
          paddingRight: 5,
          paddingLeft: 5,
          color: "white",
          marginLeft: 4,
          fontSize: 10
        }}
      >
        {extra}
      </span>
    )}
  </span>
);

const ExpandCarrot = memo(({ expanded, disabled }) => {
  const variants = {
    openDisabled: {
      d: "M770.578,215.347L399.578,586.347L26.887,213.656",
      stroke: "#333"
    },
    closedDisabled: {
      d: "M214.078,28.156L585.078,399.156L212.387,771.847",
      stroke: "#333"
    },
    openEnabled: {
      d: "M770.578,215.347L399.578,586.347L26.887,213.656",
      stroke: "#b44cd2"
    },
    closedEnabled: {
      d: "M214.078,28.156L585.078,399.156L212.387,771.847",
      stroke: "#fff"
    }
  };

  const variant =
    expanded && disabled
      ? "openDisabled"
      : expanded && !disabled
      ? "openEnabled"
      : !expanded && disabled
      ? "closedDisabled"
      : "closedEnabled";

  return (
    <span style={{ paddingTop: 4, paddingBottom: 4, paddingRight: 4 }}>
      <motion.svg
        viewBox="0 0 800 800"
        style={{
          height: 10,
          width: 10,
          fontSize: 15,
          fillRule: "evenodd",
          clipRule: "evenodd",
          strokeLinecap: "round",
          strokeLinejoin: "round",
          strokeMiterlimit: 1.5
        }}
      >
        <motion.path
          animate={variant}
          variants={variants}
          style={{
            fill: "none",
            // stroke: disabled ? theme.palette.quiet.main : theme.palette.primary.main,
            strokeOpacity: 1,
            strokeWidth: 100
          }}
        />
      </motion.svg>
    </span>
  );
});
