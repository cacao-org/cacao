'''
    pycacao.arch.confutil

    Load cacao environent variables.
'''

import os
import subprocess

from typing import Dict

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

        cacaovars_path_expect = where + '/cacaovars.bash'

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

        obj = CacaoConf(PWD=where, LOOPNAME=env_dict['CACAO_LOOPNAME'],
                        LOOPNUMBER=env_dict['CACAO_LOOPNUMBER'],
                        LOOPROOTDIR=env_dict['CACAO_LOOPROOTDIR'],
                        LOOPRUNDIR=env_dict['CACAO_LOOPRUNDIR'],
                        DMxsize=env_dict['CACAO_DMxsize'],
                        DMysize=env_dict['CACAO_DMysize'],
                        DMSPATIAL=env_dict['CACAO_DMSPATIAL'],
                        full_env=env_dict)

        return obj

    def ensure_cwd(self):
        os.chdir(self.PWD)
