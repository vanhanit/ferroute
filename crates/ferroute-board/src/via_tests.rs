// SPDX-License-Identifier: AGPL-3.0-only
use crate::{ViaKind, ViaPolicy};

#[test]
fn default_policy_is_through_only() {
    let p = ViaPolicy::default();
    assert!(p.through_only() && !p.any_span());
    assert!(p.allows(ViaKind::Through));
    for k in [ViaKind::Blind, ViaKind::Buried, ViaKind::Micro] {
        assert!(!p.allows(k), "{k:?}");
    }
}

#[test]
fn policy_from_names_is_exact() {
    let p = ViaPolicy::from_names(["blind", "micro"]).unwrap();
    assert!(!p.through && p.blind && p.micro && !p.buried && p.any_span());
    let all = ViaPolicy::from_names(ViaKind::ALL.map(ViaKind::name)).unwrap();
    assert!(all.through && all.blind && all.buried && all.micro);
    assert_eq!(ViaPolicy::from_names(["through"]).unwrap(), ViaPolicy::default());
    assert_eq!(ViaPolicy::from_names(["through", "teleport"]), Err("teleport".to_owned()));
}

#[test]
fn kind_names_roundtrip() {
    for k in ViaKind::ALL {
        assert_eq!(ViaKind::parse(k.name()), Some(k));
    }
    assert_eq!(ViaKind::parse("Through"), None);
}
