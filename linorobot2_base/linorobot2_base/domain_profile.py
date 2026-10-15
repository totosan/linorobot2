"""ROS domain contract for the supported runtime profiles."""


ROS_DOMAIN_BY_PROFILE = {
    "sim": "42",
    "replay": "43",
    "hardware": "44",
}


def resolve_ros_domain(profile, requested_domain=None, inherited_domain=None,
                       expected_profile=None):
    """Validate profile/domain selection without mutating the process environment."""
    if profile not in ROS_DOMAIN_BY_PROFILE:
        raise ValueError(f"unknown ROS profile: {profile!r}")

    if expected_profile is not None and profile != expected_profile:
        raise ValueError(
            f"this launch is restricted to profile {expected_profile!r}; got {profile!r}"
        )

    expected_domain = ROS_DOMAIN_BY_PROFILE[profile]
    if requested_domain is not None and str(requested_domain) != expected_domain:
        raise ValueError(
            f"profile {profile!r} requires ROS_DOMAIN_ID={expected_domain}; "
            f"got {requested_domain!r}"
        )

    if inherited_domain is not None and str(inherited_domain) != expected_domain:
        raise ValueError(
            f"inherited ROS_DOMAIN_ID={inherited_domain!r} conflicts with "
            f"profile {profile!r} (expected {expected_domain})"
        )

    return expected_domain