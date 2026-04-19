import type {ReactNode} from 'react';
import Footer from '@theme-original/DocItem/Footer';
import type FooterType from '@theme/DocItem/Footer';
import type {WrapperProps} from '@docusaurus/types';
import ProgressToggle from '@site/src/components/ProgressToggle';

type Props = WrapperProps<typeof FooterType>;

/**
 * Wrapping swizzle：在每個文件頁原本的 Footer（上一頁/下一頁、tags…）之前，
 * 插入 ProgressToggle。不需要逐頁手動引用元件。
 */
export default function FooterWrapper(props: Props): ReactNode {
  return (
    <>
      <ProgressToggle />
      <Footer {...props} />
    </>
  );
}
