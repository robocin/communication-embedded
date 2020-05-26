# communication-embedded

It's a library that provides a high-level interface to communicate soccer robots with a control station using nRF24l01+ module.

## Notes
Every update here affects the reposities below and its' dependents:
- [base-station](https://github.com/robocin/base-station/).
- [ssl-embedded](https://github.com/robocin/ssl-embedded/).
- [communication-software](https://github.com/robocin/communication-software/).

So, whenever changes are made at the `master`, go to the dependents repository and update the submodule:

       git submodule update --init --recursive

Make sure to update the changes into the dependent repository.
    
