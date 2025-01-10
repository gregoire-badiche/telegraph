# Telegraph

A Software Serial library for multiple parallels connections using a broadcast server

## Principle

The goal is to connect multiple Arduinos (the `modules`) around a center one (the `master`) that handles messages on multiples RX and broadcast them on a single TX, with all the modules connected in parallel on the same TX output. Ths system allows multiple connections while keeping the number of pins used manageable.

## How To

### Hardware Setup

Simply choose your master, define your `TX` în and all your `RXs` pins, and connect all your modules to the `TX` pin and a `RX` pin.

:warning: Do not forget to also connect all the grounds together for a more reliable output!

### Software setup

See the examples

### Install

Simply add this repo as a submodule inside your `lib/` folder in your PIO project, or download it and place it here.

## But Why?

I needed a way to have multiple modules communicating with one master while still performing some hardware actions with the master for a project at [@ice-efrei](https://github.com/ice-efrei) (the [ICEscapeBox](https://github.com/ice-efrei/icescapebox/) project). Multiple options were considered, but the most elegant one was to build this library.

## Contribute

You can just fork this repo and submit your changes with a detailled commit message.

### Plans for the future

This project is needing some serious documentation and comments.

I want to make the communication more reliable by adding some error correction (using parity bits).

I also want to restrict the delay (and thus the baud rate) in such a way that at higher frequencies, a 4MHz or a 16MHz card can still communicate.
