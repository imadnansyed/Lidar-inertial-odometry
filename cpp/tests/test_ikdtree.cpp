#include <ikd_Tree.h>

#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

int main()
{
    using Tree = KD_TREE<ikdTree_PointType>;

    // Allocate the large tree object on the heap.
    std::cout << "Creating iKD-Tree...\n" << std::flush;

    auto tree = std::make_unique<Tree>();

    // 1. Initialize the tree with initial points.
    Tree::PointVector points;

    points.emplace_back(0.0f, 0.0f, 0.0f);
    points.emplace_back(1.0f, 0.0f, 0.0f);
    points.emplace_back(2.0f, 0.0f, 0.0f);
    points.emplace_back(0.0f, 2.0f, 0.0f);

    tree->Build(points);

    assert(tree->validnum() == 4);

    std::cout << "Initial tree construction passed\n";

    // 2. Test nearest-neighbor search.
    Tree::PointVector nearest_points;
    std::vector<float> distances;

    tree->Nearest_Search(
        ikdTree_PointType(0.1f, 0.0f, 0.0f),
        2,
        nearest_points,
        distances,
        1.5
    );

    assert(nearest_points.size() == 2);
    assert(distances.size() == 2);

    assert(std::abs(nearest_points[0].x) < 1e-5f);
    assert(distances[0] <= distances[1]);
    assert(std::abs(distances[0] - 0.01f) < 1e-4f);

    std::cout << "Nearest-neighbor search passed\n";

    // 3. Test incremental insertion.
    Tree::PointVector new_points;

    new_points.emplace_back(4.0f, 0.0f, 0.0f);

    tree->Add_Points(new_points, false);

    assert(tree->validnum() == 5);

    nearest_points.clear();
    distances.clear();

    tree->Nearest_Search(
        ikdTree_PointType(4.1f, 0.0f, 0.0f),
        1,
        nearest_points,
        distances,
        1.0
    );

    assert(nearest_points.size() == 1);
    assert(std::abs(nearest_points[0].x - 4.0f) < 1e-5f);

    std::cout << "Incremental insertion passed\n";

    // 4. Test bounding-box deletion.
    std::vector<BoxPointType> boxes(1);

    boxes[0].vertex_min[0] = 0.9f;
    boxes[0].vertex_min[1] = -0.1f;
    boxes[0].vertex_min[2] = -0.1f;

    boxes[0].vertex_max[0] = 1.1f;
    boxes[0].vertex_max[1] = 0.1f;
    boxes[0].vertex_max[2] = 0.1f;

    const int deleted_count = tree->Delete_Point_Boxes(boxes);

    assert(deleted_count == 1);
    assert(tree->validnum() == 4);

    std::cout << "Bounding-box deletion passed\n";

    // 5. Verify deleted point cannot be found.
    nearest_points.clear();
    distances.clear();

    tree->Nearest_Search(
        ikdTree_PointType(1.0f, 0.0f, 0.0f),
        1,
        nearest_points,
        distances,
        0.2
    );

    assert(nearest_points.empty());

    std::cout << "Deletion verification passed\n";

    std::cout << "Remaining points: "
              << tree->validnum() << '\n';

    std::cout << "All iKD-Tree tests passed!\n";

    return 0;
}