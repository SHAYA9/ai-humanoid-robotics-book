import React from 'react';
import Layout from '@theme-original/DocItem/Layout';
import TranslateButton from '@site/src/components/TranslateButton/TranslateButton';
import PersonalizeButton from '@site/src/components/PersonalizeButton/PersonalizeButton';
import './CustomDocLayout.css';

export default function LayoutWrapper(props) {
  return (
    <>
      <div className="doc-customization-toolbar">
        <PersonalizeButton />
        <TranslateButton />
      </div>
      <Layout {...props} />
    </>
  );
}