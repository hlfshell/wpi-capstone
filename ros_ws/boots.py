def run():
    # First, check if the boots are visible from the current position.
    print("Checking if I see the boots from here...")
    if do_i_see("boot"):
        print("I see the boots! Moving towards them.")
        # Move to the boots
        success, msg = move_to_object("boot")
        if not success:
            # If moving to the object failed, report the error.
            print(f"Failed to move to the boots, error: {msg}")
            return fail()
    else:
        # If the boots are not visible, look around for them.
        print("I don't see the boots. I'll look around.")
        found_objects = look_around_for("boot")
        if found_objects:
            # If boots are found after looking around, move towards them.
            print("Found the boots after looking around. Moving towards them.")
            success, msg = move_to_object("boot")
            if not success:
                # If moving to the object failed after finding it, report the error.
                print(f"Failed to move to the boots after finding them, error: {msg}")
                return fail()
        else:
            # If the boots are still not found, go to the last known location.
            print("Boots not found. I'll move to the last known location in the bedroom.")
            success, msg = move_to_room("bedroom")
            if not success:
                # If moving to the bedroom failed, report the error.
                print(f"Failed to move to the bedroom, error: {msg}")
                return fail()
            # Look around once in the bedroom.
            print("In the bedroom. Looking around for the boots.")
            found_objects = look_around_for("boot")
            if not found_objects:
                # If the boots are still not found after looking in the bedroom, report failure.
                print("Boots not found in the bedroom.")
                return fail()
            else:
                print("Moving towards the boots.")
                success, msg = move_to_object("boots")
                if not success:
                    print(f"Failed to move to the boots: {msg}")
                    return fail()

    # If the boots have been located, proceed to pick them up.
    print("Picking up the boots...")
    success = pickup_object("boot")
    if not success:
        # If picking up the boots failed, report the error.
        print("Could not pick up the boots.")
        return fail()

    # After picking up the boots, move to the human to deliver them.
    print("I have the boots; taking them to the human.")
    success, msg = move_to_human()
    if not success:
        # If moving to the human failed, report the error.
        print(f"Could not navigate to human, error: {msg}")
        return fail()

    # Give the boots to the human.
    print("Giving the human the boots.")
    success = give_object("boot")
    if not success:
        # If giving the boots to the human failed, report the error.
        print("Could not give the boots to the human.")
        return fail()

    # If all steps are successful, complete the task.
    print("Task completed successfully!")
    return complete()

run()
