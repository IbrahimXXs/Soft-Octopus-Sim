def generate_robot_xml(robot_number, num_segments,
                      base_scale=1.0, base_stiffness=0.462, base_pos_z=0.02206,
                      scale_factor=0.9288627095):

    # -----------------------------------------------------
    # 1) ASSET SECTION
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
    scale_body  = base_scale

    for i in range(1, num_segments + 1):
        # compute Z
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

        # body header
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

        # six sites
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

        # prepare next
        body_indent += '    '
        stiffness  *= scale_factor
        scale_body *= scale_factor

    # -----------------------------------------------------
    # 4) CLOSE ALL THE <body> TAGS
    # -----------------------------------------------------
    for _ in range(num_segments):
        body_indent = body_indent[:-4]
        body_xml += f"{body_indent}</body>\n"

    # -----------------------------------------------------
    # 5) TENDON (SPATIAL) SECTION BETWEEN ADJACENT SEGMENTS
    # -----------------------------------------------------
    tendon_xml = ''
    for i in range(1, num_segments):
        idx = 1
        for a in (4, 5, 6):
            for b in (1, 2, 3):
                # pairs (4↔1), (5↔2), (6↔3) get the tighter range
                low = 0.003 if (a - 3) == b else 0.009
                tendon_xml += (
                    f'<spatial name="T_S{i}S{i+1}_{idx}" '
                    f'width="0.0004" frictionloss="0.1" '
                    f'rgba=".95 .3 .3 1" limited="true" '
                    f'range="{low:.3f} 0.1">\n'
                    f'    <site site="site{a}_S{i}_R{robot_number}"/>\n'
                    f'    <site site="site{b}_S{i+1}_R{robot_number}"/>\n'
                    f'</spatial>\n\n'
                )
                idx += 1

    return asset_xml + body_xml + tendon_xml


if __name__ == "__main__":
    robot_number = 1
    num_segments = 26
    xml_str = generate_robot_xml(robot_number=robot_number,
                                 num_segments=num_segments)

    # write to a .txt file
    filename = f"robot_{robot_number}_{num_segments}.txt"
    with open(filename, "w") as f:
        f.write(xml_str)

    print(f"Saved robot XML to {filename}")
