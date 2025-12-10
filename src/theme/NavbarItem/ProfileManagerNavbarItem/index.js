import React from 'react';
import { useAuth } from '@site/src/contexts/AuthContext';
import NavbarItem from '@theme/NavbarItem';

export default function ProfileManagerNavbarItem(props) {
  const { user } = useAuth();

  if (!user) {
    return null; // Don't render if not logged in
  }

  return <NavbarItem {...props} />;
}
