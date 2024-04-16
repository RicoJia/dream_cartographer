export SCH=/home/moxi/moxi_dev/./nonros/scheduler/
export MOXI_DEV=/home/moxi/moxi_dev
export M="mgoto lab2_staff_elevators"
export M2="mgoto lab1_staff_elevators"
echo '"\e[A":history-search-backward
"\e[B":history-search-forward' > ~/.inputrc
bind -f ~/.inputrc
rm -rf ~/.inputrc
PATH=$PATH:$SCH/diligent_sdk/scripts

function gcp_rico(){
    # use return instead of exit for function returns
    if [[ ${#} -lt 1 ]]; then
      echo "  This script will do git commit and git push (to the same current branch) at one shot.
Usage: ${0} <commit_msg>"
        return 1; fi

    echo ${1}
    git commit -m "${1}" --author="RicoJ <rjia@diligentrobots.com>"

    CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
    git push origin $CURRENT_BRANCH

}

function gpull_rico(){
    CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
    git pull origin $CURRENT_BRANCH
}

function gcp_rico_noverify(){
    git commit --no-verify -m "$1" --author="RicoJ <rjia@diligentrobots.com>"
    CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
    git push origin $CURRENT_BRANCH
}

function gupdate(){
    git submodule update --init --recursive
}

function gfetch_checkout_pull_rico(){   
    if [[ $# != 1 ]]
    then 
        echo "Usage: 
        gfetch_checkout_pull_rico BRANCH_NAME
        This function will then fetch the branch, check it out, and pull the most recent update"
        return 1
    fi

    git fetch
    BRANCH_NAME=$1
    git checkout $BRANCH_NAME
    # git fetch may not fetch the branch if the branch was previously deleted
    # But checkout should still work
    current_branch=$(git rev-parse --abbrev-ref HEAD)
    if [[ $current_branch != $BRANCH_NAME ]]; then echo "Switching to branch '${BRANCH_NAME}' failed. Currently on branch '${current_branch}'"; return 1; fi

     git pull origin $current_branch
}

function pip_diligent_sdk_rico(){
    cd $SCH/diligent_sdk/python
    pip install -e .

}

function prepare_workspace_rico(){

    cd $SCH/diligent_sdk/python
    gfetch_checkout_pull_rico $1
    if [[ $? == 1 ]]; then return 1; fi
    pip_diligent_sdk_rico 
    cd $MOXI_DEV
    gfetch_checkout_pull_rico $1
    if [[ $?  == 1 ]]; then return 1; fi
    build_diligent
}

function dock_rico(){
    rostopic pub /moxi/dock/goal moxi_msgs/TriggerActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  you_mad_bro: false"

}

function undock_rico(){
rostopic pub /moxi/undock/goal moxi_msgs/TriggerActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  you_mad_bro: false"


}
