import re
import os

def replace_prototypes(aadl_file, location=''):
    find_proto = re.compile('(\w+): thread (?:(\w+)::)?(\w+)\.(\w+) \(((?:\w+ => data (?:\w+::)?\w+(?:, ?)?)+)\);')
    get_proto = re.compile('(\w+) => data (?:(\w+)::)?(\w+)')

    (filename, ext) = os.path.splitext(aadl_file)

    aadl_file_mod = os.path.join(location, filename + '_mod' + ext)

    out_file = open(aadl_file_mod, 'w')
    in_file = open( os.path.join(location, aadl_file), 'r')
    pkgs = {}
    tree = {}

    for line in in_file:
        match = re.search(find_proto, line)
        if match:
            if match.group(2) not in pkgs:
                pkgs[match.group(2)] = []

            protos = match.group(5).split(',')
            proto_list = []
            type_list = []
            name_list = []
            for p in protos:
                m = re.search(get_proto, p)
                proto_list.append(m.group(1))
                type_list.append(m.group(2) + '::' + m.group(3))
                name_list.append(m.group(2) + '_' + m.group(3))
                if m.group(2) not in pkgs[match.group(2)]:
                    pkgs[match.group(2)].append(m.group(2))

            if match.group(2) not in tree:
                tree[match.group(2)] = {match.group(3): [match.group(4), [proto_list, type_list, name_list, []]]}
            else:
                if match.group(3) not in tree[match.group(2)]:
                    tree[match.group(2)][match.group(3)] = [match.group(4), [proto_list, type_list, name_list, []]]
                else:
                    if [proto_list, type_list, name_list, []] not in tree[match.group(2)][match.group(3)]:
                        tree[match.group(2)][match.group(3)].append([proto_list, type_list, name_list, []])

            str_replace = match.group(3)+'_'+'_'.join(name_list)+'.'+match.group(4)
            line = line.replace(match.group(3)+'.'+match.group(4), str_replace)
            line = line.replace('('+match.group(5)+')', '')

        out_file.write(line)

    in_file.close()
    out_file.close()

    find_with = re.compile('with ((?:\w+(?:, ?)?)+);')
    find_thread = re.compile('thread (\w+)$')
    match_proto = re.compile('(\w+): ((?:in event|out) data port) (\w+);')

    for l0 in tree:

        in_pkg_file_path = os.path.join(location, l0 + '.aadl')
        out_pkg_file_path = os.path.join(location, l0 + '_mod.aadl')

        in_pkg_file = open(in_pkg_file_path, 'r')
        out_pkg_file = open(out_pkg_file_path, 'w')

        end_file = re.compile('end '+l0+';')

        while True:
            while True:
                line = in_pkg_file.readline()
                match = re.search(find_thread, line)
                if match or re.search(end_file, line):
                    break
            if re.search(end_file, line):
                break
            if match.group(1) in tree[l0]:
                end_thread = re.compile('end ' + match.group(1) + ';')
                while True:
                    line = in_pkg_file.readline()
                    m1 = re.search(match_proto, line)
                    if m1:
                        for l in tree[l0][match.group(1)][1:]:
                            l[0] = [m1.group(1) if x == m1.group(3) else x for x in l[0]]
                            l[3].append(m1.group(2))
                    m2 = re.search(end_thread, line)
                    if m2:
                        break

        in_pkg_file.seek(0)

        out_pkg_file.write(in_pkg_file.readline())
        out_pkg_file.write(in_pkg_file.readline())
        line = in_pkg_file.readline()
        match = re.search(find_with, line)
        if match:
            ex_pkgs = match.group(1).split(',')
            for e in ex_pkgs:
                if e not in pkgs[l0]:
                    pkgs[l0].append(e)
        with_line = 'with '+', '.join(pkgs[l0]) + ';\n\n'
        out_pkg_file.write(with_line)

        for line in in_pkg_file:
            if re.search(end_file, line):
                break
            else:
                out_pkg_file.write(line)

        out_pkg_file.write('\n\n')
        for t in tree[l0]:
            for e in tree[l0][t][1:]:
                new_name = t
                for n in e[2]:
                    new_name = new_name+'_'+n
                out_pkg_file.write('\tthread '+new_name+' extends '+t+'\n')
                out_pkg_file.write('\t\tfeatures\n')
                for p, tp, po in zip(e[0], e[1], e[3]):
                    out_pkg_file.write('\t\t\t'+p+': refined to '+po+' '+tp+';\n')
                out_pkg_file.write('\tend '+new_name+';\n\n')

                impl = '.'+tree[l0][t][0]
                out_pkg_file.write('\tthread implementation '+new_name+impl+' extends '+t+impl+'\n')
                out_pkg_file.write('\tend '+new_name+impl+';\n\n')
        out_pkg_file.write(line)

        in_pkg_file.close()
        out_pkg_file.close()


replace_prototypes("example_node.aadl", "../Osate/prototype/packages/")