import os
import owncloud
import sys
import shutil
import argparse

def prepare_args():
    parser=argparse.ArgumentParser()
    parser.add_argument('-s','--save_dir',type=str,help='Path to save rosbags')
    return parser


def replicate_directory_structure(dir, local_base):
    '''
    The function takes in list of paths of directories and files present in the remote dir in cloud in variable dir.
    The function also takes the path of current working directory.
    The function then creates an identical directory structure in current working directory. If a directory is already present
    in current working directory it skips it completely.
    '''

    for item in dir:
        if item.is_dir():   # check if the current list item is a directory or not.
            local_dir = os.path.join(local_base, item.name)  # create a complete path of directory to be created
            if os.path.isdir(local_dir):
                print('Directory already exists: {} skipping...'.format(item.name))
            else:
                try:
                    os.makedirs(local_dir)
                except Exception as e:
                    print('Unable to create directory')
                print('Directroy {} created'.format(item.name))
    print('All Directories created Successfully')


def download_files(local_base,dir):

    '''
    The function takes in list of paths of directories and files present in the remote dir in cloud in variable dir.
    The function also takes the path of current working directory.
    The function then downloads all the bags present in a directory in remote directory. In a directory if a bag is already
    present it skips this bag
    '''

    for item in dir:

        dir_path=item.path
        directory,filename=os.path.split(dir_path)
        dir_parts=directory.split(os.path.sep)
        directory_name=dir_parts[-1]

        if item.is_dir():
            print('Downloading rosbags in directory {}'.format(directory_name))
            os.chdir(directory_name)
            items=oc.list(directory)
            for bags in items:
                if os.path.isfile(bags.name):
                    print('Rosbag already present')
                else:
                    download=oc.get_file(bags.path)  # oc.get_file(bag_name) downloads the bag in current working directory.
                    if download:
                        print('Rosbag {} downloaded'.format(bags.name))
                    else:
                        print('Not able to download {}'.format(bags.name))
            print('All Rosbags downloaded in {}'.format(directory_name))
            os.chdir(local_base)
            
    print('Script completed Successfully')


def main():

    global oc 
    oc=owncloud.Client('https://owncloud_server_address')
    oc.login('username','password')

    remote_dir='remote_dir_path/'
    dir=oc.list(remote_dir,depth=2)

    cwd=os.getcwd()

    replicate_directory_structure(dir,cwd)
    download_files(cwd,dir)

 #################################################################################

    # for files in dir:
    #     file=str(files)
    #     start=file.find('path=') + len('path=')
    #     end=file.find(',',start)
    #     path=file[start:end]
    #     bag_name=os.path.basename(path)
        

    #     #downloaded=oc.get_file(path)
    #     #downloaded=True
    #     if not os.path.isfile(bag_name):
    #         print('Downloading file {}'.format(bag_name))
    #         downloaded=oc.get_file(path)
    #         #downloaded=True
    #         if downloaded:
    #             print('File {} downloaded'.format(bag_name))
    #         else:
    #             print('Not able to download file')
    #     else:
    #         print('File {} already present in save directory'.format(bag_name))
###############################################################################################################



if __name__=='__main__':
    #parser=prepare_args()
    #main(parser.parse_args())
    main()