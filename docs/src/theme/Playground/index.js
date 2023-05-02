import React, {useEffect,useState} from 'react';
import clsx from 'clsx';
import useIsBrowser from '@docusaurus/useIsBrowser';
import {LiveProvider, LiveEditor, LiveError, LivePreview} from 'react-live';
import Translate from '@docusaurus/Translate';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import BrowserOnly from '@docusaurus/BrowserOnly';
import {usePrismTheme} from '@docusaurus/theme-common';
import styles from './styles.module.css';
function Header({children}) {
  return <div className={clsx(styles.playgroundHeader)}>{children}</div>;
}
function LivePreviewLoader() {
  // Is it worth improving/translating?
  // eslint-disable-next-line @docusaurus/no-untranslated-text
  return <div>Loading...</div>;
}
function ResultWithHeader() {
  return (
    <>
      <Header>
        <Translate
          id="theme.Playground.result"
          description="The result label of the live codeblocks">
          Result
        </Translate>
      </Header>
      {/* https://github.com/facebook/docusaurus/issues/5747 */}
      <div className={styles.playgroundPreview}>
        <BrowserOnly fallback={<LivePreviewLoader />}>
          {() => (
            <>
              <LivePreview />
              <LiveError />
            </>
          )}
        </BrowserOnly>
      </div>
    </>
  );
}
function ThemedLiveEditor() {
  const isBrowser = useIsBrowser();
  return (
    <LiveEditor
      // We force remount the editor on hydration,
      // otherwise dark prism theme is not applied
      key={String(isBrowser)}
      className={styles.playgroundEditor}
    />
  );
}
function EditorWithHeader() {
  return (
    <>
      <Header>
        <Translate
          id="theme.Playground.liveEditor"
          description="The live editor label of the live codeblocks">
          Live Editor
        </Translate>
      </Header>
      <ThemedLiveEditor />
    </>
  );
}

const InnerPlayground = ({children,noInline,transformCode,prismTheme,scope,playgroundPosition,...props}) => {

  const [lively, setLively] = useState(null);

  const importLively = async () => {
    const livelyPkg = await require("../../../../pkg");
    setLively(livelyPkg);
  };

  useEffect(() => {
    importLively();
  });

  return lively && (
    <div className={styles.playgroundContainer}>
        {/* @ts-expect-error: type incompatibility with refs */}
        <LiveProvider
          code={children.replace(/\n$/, '')}
          noInline={noInline}
          transformCode={transformCode ?? ((code) => `${code};`)}
          theme={prismTheme}
          scope={{...scope,lively}}
          {...props}>
          {playgroundPosition === 'top' ? (
            <>
              <ResultWithHeader />
              <EditorWithHeader />
            </>
          ) : (
            <>
              <EditorWithHeader />
              <ResultWithHeader />
            </>
          )}
        </LiveProvider>
      </div>
  )

}

export default function Playground({children, transformCode, scope, ...props}) {
  const {
    siteConfig: {themeConfig},
  } = useDocusaurusContext();
  const {
    liveCodeBlock: {playgroundPosition},
  } = themeConfig;
  const prismTheme = usePrismTheme();
  const noInline = props.metastring?.includes('noInline') ?? false;
  return (
    <BrowserOnly fallback={<div>Loading Example...</div>}>
      {()=><InnerPlayground
        children={children}
        noInline={noInline}
        transformCode={transformCode}
        prismTheme={prismTheme}
        scope={scope}
        playgroundPosition={playgroundPosition}
        {...props}
      />}
    
    </BrowserOnly>
  );
}
