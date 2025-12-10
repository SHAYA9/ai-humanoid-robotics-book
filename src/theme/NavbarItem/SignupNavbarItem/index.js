import React from 'react';
import { useAuth } from '@site/src/contexts/AuthContext';
import NavbarItem from '@theme/NavbarItem';

export default function SignupNavbarItem(props) {
  const { user } = useAuth();

  if (user) {
    return null; // Don't render if logged in
  }

  return <NavbarItem {...props} />;
}
