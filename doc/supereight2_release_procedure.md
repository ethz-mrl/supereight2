<!-- SPDX-FileCopyrightText: 2021-2025 Smart Robotics Lab, Imperial College London, Technical University of Munich -->
<!-- SPDX-License-Identifier: BSD-3-Clause -->

# supereight2 release procedure

The process to follow when making a new supereight2 release. The `main` branch
should always point to the latest release.



## Setup

Add the public repository as a remote to your local clone of the private
repository. This only needs to be done once for each local clone.

``` sh
git remote add public git@github.com:smartroboticslab/supereight2.git
```



## Releasing a new version

1. Run the test suite from
   [scripts/supereight-test.bash](./scripts/supereight-test.bash) on the `devel`
   branch and ensure there are no issues. This typically means running

   ``` sh
   ./scripts/supereight-test.bash /tmp/se2results
   ```
2. Pull the latest commits in the `devel` and `main` branches from the private
   repository remote into your local clone.

    ``` sh
    git checkout main
    git pull origin main
    git checkout devel
    git pull origin devel
    ```
3. Increase the version number in `CMakeLists.txt` in a separate commit. Keep
   [semantic versioning](https://semver.org/) in mind when doing this.

    ``` sh
    # Set the project version in CMakeLists.txt to X.Y.Z and then:
    se2_version='vX.Y.Z'
    git add CMakeLists.txt
    git commit -m "Increment version to $se2_version"
    git push origin devel
    ```
4. Fast-forward merge the `devel` branch into the `main` branch.

    ``` sh
    git checkout main
    git merge --ff devel
    ```
5. Create a git tag with the version number with name `supereight2 vX.Y.Z` and
   the output of `git shortlog` as the extended description.

    ``` sh
    tag_msg=$(printf 'supereight2 %s\n\n%s\n' "$se2_version" "$(git shortlog origin/main..main)")
    git tag --no-sign -a -m "$tag_msg" "$se2_version" main
    ```
6. Push the main branch to both the private and public repositories.

    ``` sh
    git push origin main:main
    git push public main:main
    ```
7. Push the tag to both the private and public repositories.

    ``` sh
    git push origin "$se2_version"
    git push public "$se2_version"
    ```
