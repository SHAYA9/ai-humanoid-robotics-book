import React from 'react';
import Layout from '@theme-original/DocItem/Layout';
import TranslateButton from '@site/src/components/TranslateButton/TranslateButton';

export default function LayoutWrapper(props) {
  return (
    <>
      <TranslateButton />
      <Layout {...props} />
    </>
  );
}