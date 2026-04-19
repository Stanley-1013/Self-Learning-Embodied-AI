import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';
import remarkMath from 'remark-math';
import rehypeKatex from 'rehype-katex';

const config: Config = {
  title: '學習系統',
  tagline: '機器人學、具身智能、底層演算法與數學 — 從直覺到實作',
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  url: 'https://localhost',
  baseUrl: '/',

  organizationName: 'local',
  projectName: 'robotic-learning',

  onBrokenLinks: 'warn',

  markdown: {
    format: 'detect',
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },

  i18n: {
    defaultLocale: 'zh-Hant',
    locales: ['zh-Hant', 'en'],
    localeConfigs: {
      'zh-Hant': {label: '繁體中文', direction: 'ltr', htmlLang: 'zh-Hant'},
      en: {label: 'English', direction: 'ltr', htmlLang: 'en'},
    },
  },

  headTags: [
    {
      tagName: 'script',
      attributes: {},
      innerHTML: `(function(){try{var k='locale-switch-scroll',r=sessionStorage.getItem(k);if(!r)return;var d=JSON.parse(r);if(Date.now()-d.timestamp>5000){sessionStorage.removeItem(k);return;}sessionStorage.removeItem(k);document.documentElement.style.opacity='0';window.addEventListener('DOMContentLoaded',function(){var y=d.scrollRatio*document.body.scrollHeight;window.scrollTo(0,y);requestAnimationFrame(function(){document.documentElement.style.opacity='';});});setTimeout(function(){document.documentElement.style.opacity='';},1500);}catch(e){document.documentElement.style.opacity='';}})();`,
    },
  ],

  presets: [
    [
      'classic',
      {
        docs: {
          path: 'content',
          routeBasePath: 'content',
          sidebarPath: './sidebars.ts',
          remarkPlugins: [remarkMath],
          rehypePlugins: [rehypeKatex],
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: '學習系統',
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'contentSidebar',
          position: 'left',
          label: '內容',
        },
        {
          to: '/progress',
          label: '進度總覽',
          position: 'left',
        },
        // Locale switching handled by floating LocaleToggle component (src/theme/Root.tsx)
        // instead of navbar dropdown — works in both dev and production mode
      ],
    },
    footer: {
      style: 'dark',
      copyright: `© ${new Date().getFullYear()} 學習系統 — 以 Docusaurus 構建`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
