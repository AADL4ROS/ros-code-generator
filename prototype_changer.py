import re
import os


def replace_prototypes(aadl_file, location=''):
    find_proto = re.compile(
        '(\w+):\s+thread\s+(?:(\w+)::)?(\w+)\.(\w+)\s+\(((?:\w+\s+=>\s+(?:data|subprogram)\s+(?:\w+::)?\w+(?:\s*?,\s*?)?)+)\);')
    get_proto = re.compile('(\w+)\s+=>\s+(data|subprogram)\s+(?:(\w+)::)?(\w+)')

    (filename, ext) = os.path.splitext(aadl_file)

    aadl_file_mod = os.path.join(location, filename + '_mod' + ext)

    out_file = open(aadl_file_mod, 'w')
    pkgs = {}
    tree = {}

    with open(os.path.join(location, aadl_file), 'r') as in_file:
        text = in_file.read()
        for match in find_proto.finditer(text):
            if match.group(2) not in pkgs:
                pkgs[match.group(2)] = []

            protos = match.group(5).split(',')
            proto_list = []
            type_list = []
            name_list = []
            for p in protos:
                m = re.search(get_proto, p)
                proto_list.append(m.group(1))
                type_list.append(m.group(3) + '::' + m.group(4))
                name_list.append(m.group(3) + '_' + m.group(4))
                if m.group(3) not in pkgs[match.group(2)]:
                    pkgs[match.group(2)].append(m.group(3))

            if match.group(2) not in tree:
                tree[match.group(2)] = {match.group(3): [match.group(4), [proto_list, type_list, name_list, []]]}
            else:
                if match.group(3) not in tree[match.group(2)]:
                    tree[match.group(2)][match.group(3)] = [match.group(4), [proto_list, type_list, name_list, []]]
                else:
                    if [proto_list, type_list, name_list, []] not in tree[match.group(2)][match.group(3)]:
                        tree[match.group(2)][match.group(3)].append([proto_list, type_list, name_list, []])

            text = re.sub(
                re.compile('(' + match.group(3) + ')\.(' + match.group(4) + ')\s+\(' + match.group(5) + '\);'),
                match.group(3) + '_' + '_'.join(name_list) + '.' + match.group(4) + ';', text)

        out_file.write(text)

    in_file.close()
    out_file.close()

    find_with = re.compile('with\s+((?:\w+(?:\s*,\s*?)?)+);')
    find_thread = re.compile('thread\s+(?P<type>\w+)\s+prototypes[\s\S]+end\s+(?P=type);')
    match_proto = re.compile('(\w+):\s+((?:in\s+event|out)\s+data\s+port|provides\s+subprogram\s+access)\s+(\w+)\s*;')

    for l0 in tree:

        in_pkg_file_path = os.path.join(location, l0 + '.aadl')
        out_pkg_file_path = os.path.join(location, l0 + '_mod.aadl')

        out_pkg_file = open(out_pkg_file_path, 'w')

        with open(in_pkg_file_path, 'r') as in_pkg_file:
            text = in_pkg_file.read()
            match = re.search(find_with, text)
            if match:
                ex_pkgs = match.group(1).split(',')
                for e in ex_pkgs:
                    if e not in pkgs[l0]:
                        pkgs[l0].append(e)
            with_line = 'with ' + ', '.join(pkgs[l0]) + ';\n\n'
            text = re.sub(re.compile('public(?:\s+with\s+((?:\w+(?:\s*,\s*?)?)+);)?'), 'public\n\t' + with_line, text)

            for thread_match in find_thread.finditer(text):
                for proto_match in match_proto.finditer(thread_match.group(0)):
                    if thread_match.group('type') in tree[l0]:
                        for l in tree[l0][thread_match.group('type')][1:]:
                            l[0] = [proto_match.group(1) if x == proto_match.group(3) else x for x in l[0]]
                            l[3].append(proto_match.group(2))
                thread_block = re.sub(re.compile('provides\s+subprogram\s+access\s+\w+\s*;'),
                                      'provides subprogram access;', thread_match.group(0))
                text = re.sub(thread_match.group(0), thread_block, text)

            text = re.sub(re.compile('end ' + l0 + ';'), '', text)

            out_pkg_file.write(text)

        out_pkg_file.write('\n\n')
        for t in tree[l0]:
            for e in tree[l0][t][1:]:
                new_name = t
                for n in e[2]:
                    new_name = new_name + '_' + n
                out_pkg_file.write('\tthread ' + new_name + ' extends ' + t + '\n')
                out_pkg_file.write('\t\tfeatures\n')
                for p, tp, po in zip(e[0], e[1], e[3]):
                    out_pkg_file.write('\t\t\t' + p + ': refined to ' + po + ' ' + tp + ';\n')
                out_pkg_file.write('\tend ' + new_name + ';\n\n')

                impl = '.' + tree[l0][t][0]
                out_pkg_file.write('\tthread implementation ' + new_name + impl + ' extends ' + t + impl + '\n')
                out_pkg_file.write('\tend ' + new_name + impl + ';\n\n')
        out_pkg_file.write('end ' + l0 + ';')

        in_pkg_file.close()
        out_pkg_file.close()
