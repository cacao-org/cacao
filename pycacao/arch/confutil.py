'''
    pycacao.arch.confutil

    Load cacao environent variables.
'''

import os
import subprocess

from typing import Dict, Tuple

from dataclasses import dataclass


@dataclass
class CacaoConf:
    LOOPNAME: str
    LOOPNUMBER: int

    PWD: str
    LOOPROOTDIR: str
    LOOPRUNDIR: str

    DMxsize: int
    DMysize: int
    DMSPATIAL: bool

    full_env: Dict[str, str]

    @classmethod
    def from_pwd_tree(self, where: str = '.'):
        '''
        Generate a CacaoConf object from cacaovars.bash
        found under the 'where' directory.
        '''

        abs_path = os.path.abspath(where)
        cacaovars_loopname_file = abs_path + '/LOOPNAME'
        with open(cacaovars_loopname_file, 'r') as f:
            CACAO_LOOPNAME = f.read().strip()

        cacaovars_path_expect = abs_path + f'/cacaovars.{CACAO_LOOPNAME}.bash'

        has_cacaovars = os.path.isfile(cacaovars_path_expect)
        if not has_cacaovars:
            raise FileNotFoundError(f'Cannot locate {cacaovars_path_expect}')

        cmd = f'echo $(source {cacaovars_path_expect} > /dev/null; env)'
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True,
                             executable='bash')

        env_list = p.stdout.readlines()[0].strip().split()
        env_dict = {}
        for env_line in env_list:
            split = env_line.decode().split('=')
            if split[0].startswith('CACAO_'):
                env_dict[split[0]] = split[1]

        obj = CacaoConf(PWD=abs_path, LOOPNAME=env_dict['CACAO_LOOPNAME'],
                        LOOPNUMBER=env_dict['CACAO_LOOPNUMBER'],
                        LOOPROOTDIR=env_dict['CACAO_LOOPROOTDIR'],
                        LOOPRUNDIR=env_dict['CACAO_LOOPRUNDIR'],
                        DMxsize=int(env_dict['CACAO_DMxsize']),
                        DMysize=int(env_dict['CACAO_DMysize']),
                        DMSPATIAL=bool(int(env_dict['CACAO_DMSPATIAL'])),
                        full_env=env_dict)

        return obj

    def ensure_cwd(self):
        os.chdir(self.PWD)

    def get_dm_pup_params(self) -> Tuple[float, float, float]:
        if 'CACAO_DM_beam_xcent' in self.full_env:
            assert 'CACAO_DM_beam_ycent' in self.full_env
            assert 'CACAO_DM_beam_rad' in self.full_env

            return (float(self.full_env['CACAO_DM_beam_xcent']),
                    float(self.full_env['CACAO_DM_beam_ycent']),
                    float(self.full_env['CACAO_DM_beam_rad']))

        else:
            print('Using defaults in get_dm_pup_params')
            # Per cacao-mkDMpokemodes... but this needs a 0.5 offset.
            return (self.DMxsize / 2., self.DMysize / 2., self.DMxsize * 0.45)

    def get_dm_size(self) -> Tuple[int, int]:
        return (self.DMxsize, self.DMysize)
