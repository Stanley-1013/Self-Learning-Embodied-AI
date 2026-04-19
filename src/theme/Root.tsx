import type {ReactNode} from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import LocaleToggle from '@site/src/components/LocaleToggle';

interface Props {
  children: ReactNode;
}

/**
 * Root wrapper — injected below <html> and above everything else.
 * We use this to mount global floating UI (locale toggle) on every page.
 */
export default function Root({children}: Props): ReactNode {
  return (
    <>
      {children}
      <BrowserOnly>{() => <LocaleToggle />}</BrowserOnly>
    </>
  );
}
