Contributing to quic-usb-drivers

Hi there! We’re thrilled that you’d like to contribute to this project. Your help is essential for keeping this project great and for making it better.

Branching Strategy
In general, contributors should develop on branches based off of main and pull requests should be made against main.

Submitting a pull request
Please read our code of conduct and license.

Fork and clone the repository.

git clone https://github.com/quic/quic-usb-drivers.git
Create a new branch based on main:

git checkout -b <my-branch-name> main
Create an upstream remote to make it easier to keep your branches up-to-date:

git remote add upstream https://github.com/quic/<REPLACE-ME>.git
Make your changes, add tests, and make sure the tests still pass.

Commit your changes using the DCO. You can attest to the DCO by commiting with the -s or --signoff options or manually adding the "Signed-off-by":

git commit -s -m "Really useful commit message"`
After committing your changes on the topic branch, sync it with the upstream branch:

git pull --rebase upstream main
Push to your fork.

git push -u origin <my-branch-name>
The -u is shorthand for --set-upstream. This will set up the tracking reference so subsequent runs of git push or git pull can omit the remote and branch.

Submit a pull request from your branch to main.

Pat yourself on the back and wait for your pull request to be reviewed.

Here are a few things you can do that will increase the likelihood of your pull request to be accepted:

Follow the existing style where possible. 
Write tests.
Keep your change as focused as possible. If you want to make multiple independent changes, please consider submitting them as separate pull requests.
Write a good commit message.
It's a good idea to arrange a discussion with other developers to ensure there is consensus on large features, architecture changes, and other core code changes. PR reviews will go much faster when there are no surprises.