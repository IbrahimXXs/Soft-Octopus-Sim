def generate_robot_xml(robot_number, num_segments,
                      base_scale=1.0, base_stiffness=0.462, base_pos_z=0.02206,
                      scale_factor=0.9288627095):

    # -----------------------------------------------------
    # 1) ASSET SECTION (unchanged)
    # -----------------------------------------------------
    asset_xml = '    <asset>\n'
    scale = base_scale
    for i in range(1, num_segments + 1):
        mesh_name = f"segment{i}N_mesh"
        scale_str = f"{scale:.9f} {scale:.9f} {scale:.9f}"
        asset_xml += (
            f'\t<mesh name="{mesh_name}" '
            f'file="3D_OctV2/meter/3dt2_1_meter.STL" scale="{scale_str}"/>\n'
        )
        scale *= scale_factor
    asset_xml += '    </asset>\n\n'

    # -----------------------------------------------------
    # 2) DEFINE BASE SITE POSITIONS & SIZE
    # -----------------------------------------------------
    # these are the S1_R1 positions & size from your example
    base_sites = [
        (0.013190,  0.007590, 0.006),    # site1
        (0.000000, -0.015190, 0.006),    # site2
        (-0.013190, 0.007590, 0.006),    # site3
        (0.012690,  0.007301, 0.020),    # site4
        (0.000000, -0.014614, 0.020),    # site5
        (-0.012689, 0.007302, 0.020),    # site6
    ]
    base_size = 0.001

    # -----------------------------------------------------
    # 3) BODY + SITE SECTION
    # -----------------------------------------------------
    stiffness   = base_stiffness
    body_indent = ''
    body_xml    = ''
    prev_raw_z  = None

    # we'll track a separate scale for sites & mesh
    scale_body = base_scale

    for i in range(1, num_segments + 1):
        # --- compute Z position ---
        if i == 1:
            raw_z = 0.0
        elif i == 2:
            raw_z = base_pos_z
        else:
            raw_z = prev_raw_z * scale_factor
        prev_raw_z = raw_z
        pos_z = raw_z + 0.001
        pos   = f"0 0 {pos_z:.5f}"

        seg_name   = f"segment_{i}_R{robot_number}"
        mesh_name  = f"segment{i}N_mesh"
        joint_name = f"J{i}_R{robot_number}"

        # --- body, geom, joint, inertial ---
        body_xml += f'{body_indent}<body name="{seg_name}" pos="{pos}" euler="0 0 0">\n'
        body_xml += f'{body_indent}    <geom type="mesh" mesh="{mesh_name}" rgba="1 1 1 1"/>\n'
        body_xml += (
            f'{body_indent}    <joint name="{joint_name}" '
            f'pos="0 0 0" axis="1 0 0" type="ball" '
            f'stiffness="{stiffness:.9f}" damping="0.02"/>\n'
        )
        body_xml += (
            f'{body_indent}    <inertial pos="0.011 0 0" quat="1 0 0 0" '
            f'mass="0.01" diaginertia="0.000083 0.000083 0.00005"/>\n\n'
        )

        # --- now emit the six sites, scaled by scale_body ---
        for j, (bx, by, bz) in enumerate(base_sites, start=1):
            x = bx * scale_body
            y = by * scale_body
            z = bz * scale_body
            s = base_size * scale_body
            site_name = f"site{j}_S{i}_R{robot_number}"
            body_xml += (
                f'{body_indent}    <site name="{site_name}" '
                f'pos="{x:.6f} {y:.6f} {z:.6f}" '
                f'size="{s:.6f} {s:.6f} {s:.6f}"/>\n'
            )
        body_xml += '\n'

        # --- prepare for next segment ---
        body_indent += '    '
        stiffness  *= scale_factor
        scale_body *= scale_factor

    # -----------------------------------------------------
    # 4) CLOSE ALL THE <body> TAGS
    # -----------------------------------------------------
    for _ in range(num_segments):
        body_indent = body_indent[:-4]
        body_xml += f"{body_indent}</body>\n"

    return asset_xml + body_xml


if __name__ == "__main__":
    xml_str = generate_robot_xml(robot_number=1, num_segments=26)
    print(xml_str)
